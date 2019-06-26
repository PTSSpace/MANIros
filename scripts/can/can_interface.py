
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
class CAN_Listener(can.Listener):
    """CAN Listener class to catch encoder odometry feedback"""
    def __init__(self):
        # Bus feedback variables
        self.activity           = [0, 0, 0, 0, 0]
        # Electrical Power Supply (EPS)
        self.current            = [.0, .0, .0, .0, .0]
        self.epsPowerQueue      = Queue.Queue(maxsize=10)           # EPS power switch toggled
        self.epsMsgQueue        = Queue.PriorityQueue(maxsize=10)   # EPS critical current and over-current warning
        self.count              = 0                                 # Priority queue counter
        # Locomotion Control (LC)
        self.lcSteer = threading.Event()                            # Steering completed feedback
        #self.steer              = [0, 0, 0, 0]
        #self.orientation        = [.0, .0, .0, .0]
        self.pulses             = [0, 0, 0, 0]
        self.revolutions        = [0, 0, 0, 0]
        self.lcMsgQueue         = Queue.Queue(maxsize=10)           # Orientation reached feedback


        #self.epsMsgProcessing = threading.Thread(name = "epsMsgProcessingThread" target=self.eps_message_processing, args=(epsMsgQueue,))


    def on_message_received(self, rxMsg):
        ID = rxMsg.arbitration_id
        print("CI \t Message received \t ID:%d" % ID )
        try:
            # Electrical Power Supply (EPS)
            if ID == errorWrn:
                sensor_error = [0, 0, 0, 0]
                for idx in range (0, len(currentSensorIndex)):
                    sensor_error[idx] = struct.unpack('?', rxMsg.data[idx:idx+1])[0]
                self.epsMsgQueue.put([1, self.count, sensor_error], block=False)
                self.count += 1
            elif ID == currentWrn:
                crit_current = [0, 0, 0, 0]
                for idx in range (0, len(currentSensorIndex)):
                    crit_current[idx] = struct.unpack('?', rxMsg.data[idx:idx+1])[0]
                self.epsMsgQueue.put([2, self.count, crit_current], block=False)
                self.count += 1
            elif ID == powerFd:
                motorPower = struct.unpack('?', rxMsg.data[0:1])[0]
                self.epsPowerQueue.put(motorPower, block=False)
            elif ID == currentFd:
                idx = struct.unpack('i', rxMsg.data[0:4])[0]
                if idx ==1:
                    self.current[idx] = CAN_Listener.unwrap_message_format(struct.unpack('i', rxMsg.data[4:8])[0], 2)
                elif idx < len(self.current):
                    self.current[idx] = CAN_Listener.unwrap_message_format(struct.unpack('i', rxMsg.data[4:8])[0],3)
                else:
                    print ("CI \t Message Error")
                self.activity[0] = 1

            # Locomotion Control (LC)
            elif ID in locomotionFb:
                idx =  [int(x) for x in locomotionFb].index(ID)
                flag = struct.unpack('?', rxMsg.data[0:1])[0]
                print("CI \t Message Source: %s \t Locomotion completed: %d" % (wheelIndex[idx], flag))
                message = [idx, flag]
                self.lcMsgQueue.put(message, block=False)
                # Update node activity flag
                self.activity[idx+1] = 1
            elif ID in odometryFb:
                idx =  [int(x) for x in odometryFb].index(ID)
                self.pulses[idx] = struct.unpack('i', rxMsg.data[0:4])[0]
                self.revolutions[idx] = struct.unpack('i', rxMsg.data[4:8])[0]
                print("CI \t Message Source: %s \t Pulses: %d \t Revolutions: %d" % (wheelIndex[idx], self.pulses[idx], self.revolutions[idx]))
                # Update node activity flag
                self.activity[idx+1] = 1
            else:
                print("CI \t Message ID %d not in known list" % ID)
        except Queue.Full:
            print("CI \t Message queue full")


    @staticmethod
    def unwrap_message_format(value, type):
        # Scale to float number [0..1]
        value = float(value) /MAX_VALUE
        if type == 0:
            value *= MAX_ORT
        elif type == 1:
            value *= MAX_VEL
        elif type == 2:
            value *= MAX_CUR_B
        elif type == 3:
            value *= MAX_CUR_M
        return value


class CANInterface():
    """Interface class for CAN bus"""
    @classmethod
    def __init__(cls):
        # Mutex lock to protect CAN interface
        cls.lock = threading.Lock()
        # Start up CAN bus
        cls.bus = can.interface.Bus(bustype='socketcan', channel='can0', bitrate=BITRATE)

        # Create listener
        cls.listener = CAN_Listener()
        # Add notifyier to call listener periodically
        with cls.lock:
            cls.notifier = can.Notifier(cls.bus, [cls.listener])

    @classmethod
    def send_can_message(cls, arbitration_id, values):
        data = b''
        # Convert to bytes
        for x in values:
            # Check data type
            if type(x) == int:
                data = data + struct.pack('i',x)
            elif type(x) == bool:
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
