#!/usr/bin/python

import rospy
import time

from geometry_msgs.msg import Twist
from maniros.msg import MoveControl # Speed based control
# from maniros.msg import RoverControl # Distance based control

class CmdVel:
    def __init__(self):
        self.pub = rospy.Publisher("cmd_vel", MoveControl, queue_size=10)
        rospy.Subscriber("move_base/cmd_vel", Twist, callback=self.on_autonomous_cmd)
        rospy.Subscriber("teleop/cmd_vel", Twist, callback=self.on_human_cmd)
        self.block_duration = 0
        self.human_cmd_time = time.time()

    def on_autonomous_cmd(self, data):
        time_since_human_cmd = time.time() - self.human_cmd_time
        if time_since_human_cmd >= self.block_duration:
            self.block_duration = 0 # stop blocking
            self.message_publish(data)

    def on_human_cmd(self, data):
        self.human_cmd_time = time.time()
        self.block_duration = 5
        self.message_publish(data)

    def message_publish(self, data):
        msg = MoveControl()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "/cmd_vel";
        msg.xSpeed = data.linear.x
        msg.ySpeed = data.linear.y          
        msg.rotationAngle = data.angular.z     

        rospy.loginfo("Cmd_vel \t x:%f y:%f rot:%f" % (msg.xSpeed, msg.ySpeed, msg.rotationAngle));
        self.pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node("cmd_vel")
    muxer = CmdVel()
    rospy.spin()