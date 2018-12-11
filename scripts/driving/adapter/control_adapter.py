#!/usr/bin/env python
import rospy
from maniros.msg import RoverControl
from maniros.msg import MotorControl

pub = rospy.Publisher("motor_control", MotorControl, queue_size=10)

def callback(data):
    rospy.loginfo("Adapter: I've heard x:%d y:%d rot:%d - translating..." % (data.xDistance, data.yDistance, data.rotationDistance));
    msg = translateRoverControl(data);
    pub.publish(msg);
    rospy.loginfo("Adapter: I've sent fr:%d fl:%d rl:%d rr:%d" % (msg.front_right_angle, msg.front_left_angle, msg.rear_left_angle, msg.rear_right_angle));

def listener():
    rospy.init_node("control_adapter", anonymous=True);
    sub = rospy.Subscriber("rover_control", RoverControl, callback)
    rospy.spin()

def translateRoverControl(data):
    msg = MotorControl();

    # Fake Vector-Protocol implementation
    msg.front_right_angle = data.xDistance + 1;
    msg.front_left_angle = data.xDistance + 2;
    msg.rear_left_angle = data.xDistance + 3;
    msg.rear_right_angle = data.xDistance + 4;

    return msg;

if __name__ == '__main__':
    listener();