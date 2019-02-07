#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

# initilizes and saves the state of the deadman switch (button RB)
deadman = 0

def callback(data):
    global deadman
    twist = Twist()
    abort = Twist()
    twist.linear.x = data.axes[1]
    twist.linear.y = data.axes[0]
    twist.angular.z = data.axes[2]
    abort.linear.x = 0
    abort.linear.y = 0
    abort.angular.z = 0
    if (data.buttons[5]):
	deadman = 1
        rospy.loginfo("Input: Deadman switch is:%d" % deadman); 
        pub.publish(twist)
        rospy.loginfo("Input: I've sent x:%f y:%f rot:%f" % (twist.linear.x, twist.linear.y, twist.angular.z));

    if (data.buttons[5] == 0 and deadman != 0):
	deadman = 0
	rospy.loginfo("Input: Deadman switch is:%d" % deadman);
	pub.publish(abort)
        rospy.loginfo("Input: Abort message sent")

    if (data.buttons[4]):
        pub.publish(abort)
        rospy.loginfo("Input: Abort message sent")

# Intializes everything
def start():
    # publishing twist message to "/cmd_vel"
    global pub
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    # subscribed to joystick inputs on topic "joy"
    rospy.Subscriber("joy", Joy, callback)
    # starts the node
    rospy.init_node('teleop_input')

    rospy.spin()

if __name__ == '__main__':
    start()

