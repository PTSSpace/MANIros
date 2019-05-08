from __future__ import print_function
import can
import struct
import time

RX_ID = 0x100
TX_ID = 0x101

if __name__ == "__main__":
    #send_one()

    bus = can.interface.Bus(bustype='socketcan', channel='can1', bitrate=500000)

    listener = can.BufferedReader()

    count = 2

    try:

        # Wait for data
        print ("waiting for CAN message")
        msg = bus.recv()
        print ("message received")
        listener(msg)
        r_msg = listener.get_message()
        print (r_msg.arbitration_id)
        print (struct.unpack('i', r_msg.data[0:4])[0])
        print (struct.unpack('i', r_msg.data[4:8])[0])

        while True:
            time.sleep(1)
            if (count > 5):
                count = 0
                # Send speed data
                speed = int(1)*1000
                orientation = int(90)*1000
                speedB = struct.pack('i',speed)
                orientationB = struct.pack('i',orientation)
                msg = can.Message(arbitration_id=257,
                                  dlc= 8, data=speedB+orientationB,
                                  extended_id=False)
                try:
                    bus.send(msg)
                    print("Message sent on {}".format(bus.channel_info))
                except can.CanError:
                    print("Message NOT sent")

                time.sleep(1)
                pass
            count += 1
    except KeyboardInterrupt:
        listener.stop()