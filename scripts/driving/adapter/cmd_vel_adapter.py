#!/usr/bin/env python
import rospy
from maniros.msg import RoverControl
from geometry_msgs.msg import Twist


def callback(data):
    #rospy.loginfo(data.linear.x)
    msg = RoverControl()
    msg.sequenceCount = 1
    msg.mode = 0
    msg.xDistance = data.linear.x * 1000
    msg.yDistance = data.linear.y * 1000
    msg.rotationDistance = data.angular.z * 1000
    msg.time = 10
    rospy.loginfo("Rover Control: I've heard x:%d y:%d rot:%d" % (data.linear.x * 1000, data.linear.y * 1000, data.angular.z * 1000));
    rospy.loginfo("Rover Control: I've sent x:%d y:%d rot:%d" % (msg.xDistance, msg.yDistance, msg.rotationDistance));

    pub.publish(msg)


def listener():
    global pub
    rospy.init_node("cmd_vel_adapter", anonymous=True);
    
    pub = rospy.Publisher("rover_control", RoverControl, queue_size=10)
    sub = rospy.Subscriber("cmd_vel", Twist, callback)

    rospy.spin()


if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException: pass
