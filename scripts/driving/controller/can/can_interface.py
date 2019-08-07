"""
This program provides a CAN bus interface.
It connects to the can0 interface and provides options for sending and receiving messages
within the defined CAN protocol.
The interface is comprised of the main interface class which is used for message passing
and the listener class that is an interrupt driven class to record messages from the bus.

All values contained in a message are either booleans representing state switch information
or int16 values bounded to [-32768 ... 32768]. Integer values passed to or received
from the bus are given in the various formats.

Current sensor feedback from the EPS is normed values to fit an int16 [-32768 ... 32768]
and must therefore be scaled in the monitoring node.

The locomotion control uses velocity or orientation values in either encoder pulses
or pulses per second and counter values for drive encoder rotations. These values must
not be scaled but possibly converted inot more usable units in the monitoring node
for futher processing.

Received messages are passed into interrupt Queues and Events that can be read from
the monitoring node.


CANInterface:
    * CAN bus interface and message passing
CANListener:
    *   CAN bus interrupt handler
    Events and Event Queues:
        *   epsPowerQueue   - EPS power switch toggled
        *   epsMsgQueue     - EPS critical current and over-current warning
        *   lcMsgQueue      - Orientation reached feedback
        #*   lcSteer         - Steering completed feedback
"""

"""
Imports
"""
from can_protocol import *
import threading
import Queue
import can
import struct

"""
Classes
"""
class CANListener(can.Listener):
    """CAN Listener class to receive can node feedback"""
    def __init__(self):
        # Bus feedback variables
        self.activity           = [0, 0, 0, 0, 0]                   # CAN node activity
        # Electrical Power Supply (EPS)
        self.current            = [.0, .0, .0, .0, .0]
        self.epsPowerQueue      = Queue.Queue(maxsize=10)           # EPS power switch toggled
        self.epsMsgQueue        = Queue.PriorityQueue(maxsize=10)   # EPS critical current and over-current warning
        self.count              = 0                                 # Priority queue counter
        # Locomotion Control (LC)
        #self.lcSteer            = threading.Event()                 # Steering completed feedback
        self.orientation        = [0, 0, 0, 0]                      # Steer motor orientation [pulses]
        self.velocity           = [0, 0, 0, 0]                      # Drive motor velocity [pulses/s]
        self.pulses             = [0, 0, 0, 0]                      # Drive motor orientation [pulses]
        self.revolutions        = [0, 0, 0, 0]                      # Drive motor revolutions
        self.lcMsgQueue         = Queue.Queue(maxsize=10)           # Orientation reached feedback

    def on_message_received(self, rxMsg):
        """
        Listens on CAN bus. Receives messages from the bus.
        Extracts message type, given by the arbitration_id.
        Interprets the message data and passes it into
        interrupt queues for the monitoring node to process.
        :param rxMsg: Reveiced CAN message (can.Message object)
        """
        # Get ID for processing according to message type
        ID = rxMsg.arbitration_id
        print("CI \t Message received \t ID:%d" % ID )

        # Message data handling
        try:
            # Electrical Power Supply (EPS)
            if ID == errorWrn:                  # Overcurrent or general sensor error warning
                sensor_error = [0, 0, 0, 0]
                for idx in range (0, len(currentSensorIndex)):
                    sensor_error[idx] = struct.unpack('?', rxMsg.data[idx:idx+1])[0]
                self.epsMsgQueue.put([1, self.count, sensor_error], block=False)
                self.count += 1
            elif ID == currentWrn:              # Critical current warning
                crit_current = [0, 0, 0, 0]
                for idx in range (0, len(currentSensorIndex)):
                    crit_current[idx] = struct.unpack('?', rxMsg.data[idx:idx+1])[0]
                self.epsMsgQueue.put([2, self.count, crit_current], block=False)
                self.count += 1
            elif ID == powerFb:                 # Feedback for power switch states
                motorPower = struct.unpack('?', rxMsg.data[0:1])[0]
                self.epsPowerQueue.put(motorPower, block=False)
            elif ID == currentFb:               # Feedback for current sensors
                idx = struct.unpack('i', rxMsg.data[0:4])[0]
                if idx ==1:
                    self.current[idx] = CANListener.unwrap_message_format(struct.unpack('i', rxMsg.data[4:8])[0], 2)
                elif idx < len(self.current):
                    self.current[idx] = CANListener.unwrap_message_format(struct.unpack('i', rxMsg.data[4:8])[0],3)
                else:
                    print ("CI \t Message Error")
                self.activity[0] = 1

            # Locomotion Control (LC)
            elif ID in locomotionFb:            # Feedback for LC command completed
                # Get controller node index/location
                idx =  [int(x) for x in locomotionFb].index(ID)
                # Extract flag feedback data
                flag = struct.unpack('?', rxMsg.data[0:1])[0]
                print("CI \t Message Source: %s \t Locomotion completed: %d" % (wheelIndex[idx], flag))
                # Pass message into processing queue for monitoring node
                message = [idx, flag]
                self.lcMsgQueue.put(message, block=False)
                # Update wheel controller activity flag
                self.activity[idx+1] = 1
            elif ID in odometryFb:              # Feedback for LC odometry
                # Get controller node index/location
                idx =  [int(x) for x in odometryFb].index(ID)
                # Extract odometry data
                self.orientation[idx] = struct.unpack('i', rxMsg.data[0:2])[0]
                self.velocity[idx] = struct.unpack('i', rxMsg.data[2:4])[0]
                self.pulses[idx] = struct.unpack('i', rxMsg.data[4:6])[0]
                self.revolutions[idx] = struct.unpack('i', rxMsg.data[6:8])[0]
                print("CI \t Message Source: %s \t Orientation: %d \t Velocity: %d \t Pulses: %d \t Revolutions: %d"
                    % (wheelIndex[idx], self.orientation[idx], self.velocity[idx], self.pulses[idx], self.revolutions[idx]))
                # Update wheel controller activity flag
                self.activity[idx+1] = 1
            else:
                print("CI \t Message ID %d not in known list" % ID)
        except Queue.Full:
            print("CI \t Message queue full")

    @staticmethod
    def unwrap_message_format(value, type):
        # Scale to float number [-1..1]
        value = float(value) /MAX_VALUE
        return value

    @staticmethod
    def wrap_message_format(value):
        # Scale to integer number [-MAX_VALUE..MAX_VALUE]
        value = int(value) *MAX_VALUE
        return value


class CANInterface():
    """Interface class for CAN bus to send and receive messages"""
    @classmethod
    def __init__(cls):
        # Mutex lock to protect CAN interface
        cls.lock = threading.Lock()
        # Start up CAN bus
        cls.bus = can.interface.Bus(bustype='socketcan', channel='can0', bitrate=BITRATE)

        # Create listener
        cls.listener = CANListener()
        # Add notifyier to call listener periodically
        with cls.lock:
            cls.notifier = can.Notifier(cls.bus, [cls.listener])

    @classmethod
    def send_can_message(cls, arbitration_id, values):
        """
        Send message on can bus.
        :param arbitration_id: CAN message ID containing node ID and message type
        :param values: List of values to be sent in the CAN message
        """
        data = b''
        # Convert to bytes
        for x in values:
            # Check data type
            if type(x) == int:
                # Pack as signed short int (2 bytes)
                data = data + struct.pack('h',x)
            elif type(x) == bool:
                # Pack as bool/char (1 byte)
                data = data + struct.pack('?',x)
        # Create CAN message
        txMsg = can.Message(arbitration_id=arbitration_id,
                      dlc= len(data), data=data,
                      extended_id=False)
        # Send CAN message
        with cls.lock:
            try:
                cls.bus.send(txMsg)
                print("CI \t Message sent on {}".format(cls.bus.channel_info))
                return 1
            except can.CanError:
                print("CI \t Message NOT sent")
                return 0
