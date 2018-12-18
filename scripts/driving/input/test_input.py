#!/usr/bin/env python
import rospy
from maniros.msg import RoverControl

def talker():
    pub = rospy.Publisher("rover_control", RoverControl, queue_size=10)
    rospy.init_node("random_input", anonymous=True)
    r = rospy.Rate(10) #10hz

    msg = RoverControl()
    msg.sequenceCount = 1
    msg.mode = 0
    msg.xDistance = 0
    msg.yDistance = 0
    msg.rotationDistance = 1
    msg.time = 10

    while not rospy.is_shutdown():
        rospy.loginfo("Rover Control: I've sent x:%d y:%d rot:%d" % (msg.xDistance, msg.yDistance, msg.rotationDistance));
        pub.publish(msg)
        r.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass