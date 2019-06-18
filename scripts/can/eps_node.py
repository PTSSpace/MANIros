#!/usr/bin/python
from __future__ import print_function


"""
This program provides a ROS node for the can0 bus interface.
It connects the electrical power supply node to the ROS network.
Power commands are forwarded to the drive nodes.
Encoder odometry messages are received from the drive nodes and published to the ROS network.

ROS actions:
* eps_control
Published ROS topics:
*   eps_current
"""

"""
Imports
"""
import rospy
import actionlib
import time
import math
import Queue

# Import CAN protocol parameters
from can_protocol import *
# Import CAN interface
from can_interface import CANInterface

# Import ROS messages
from maniros.msg import EpsCurrent                                                 	# Electrical power supply measured currents
# Electrical power supply control action
from maniros.msg import EpsAction
from maniros.msg import EpsFeedback
from maniros.msg import EpsResult

"""
Classes
"""

class LocomotionControl(object):
    """Power control/monitoring server for CAN bus communication with EPS"""
    _feedback = EpsFeedback()
    _result = EpsResult()

    def __init__(self, name):
        # Switch states
        self.epsInitialised		= False
        self.motorPower 		= False
        self.publisherMode 		= False

        # Construct CAN bus interface
        self.ci = CANInterface()

        # EPS electrical current publisher
        self.eps_pub = rospy.Publisher("eps_current", EpsCurrent, queue_size=10)
        # Subscribe to eps state commands
        self.switch_sub = rospy.Subscriber("teleop/eps_switch", MoveCommand, self.locomotion_switch, queue_size=10)

        # Start ROS publisher for eps current meassurements
        self.timer = rospy.Timer(rospy.Duration(10), self.CAN_subscriber)

        # Locomotion control action
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, EpsAction, execute_cb=self.eps_switch, auto_start = False)
        self._as.start()

    def eps_switch(self, data):
	# EPS command action
        # Helper variables
        r = rospy.Rate(100)
	# Append message CAN bus message feedback
        self._feedback.sequence = []
        rospy.loginfo("Command \t Motors:%d" % (data.MotorPower))
        # Toggle motor power switch
        if (data.MotorPower):
            self.motorPower = not ci.listener.self.motorPower
            # Motors start/stop command
            # Send CAN EPS command
            rospy.loginfo("Command \t %s wheel \t Motors:%d" % ('eps', self.motorPower))
            sent = self.ci.send_can_message(powerCmd, [self.motorPower])
            self._feedback.sequence.append(sent)
        rospy.loginfo('%s: Waiting EPS feedback' % (self._action_name))

        while (True):
        	try:
                self.motorPower = self.ci.listener.epsPowerQueue.get(block=False)
                self.ci.listener.epsMsgQueue.task_done()
                break
            except Queue.Empty:
                r.sleep()
                # rospy.loginfo("Message queue empty")
            if (self._as.is_preempt_requested() or rospy.is_shutdown()):
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                success = False
                break
        return success

    def get_eps_currents(self):
        # Get message values from listener
        msg = EpsCurrent(*(self.ci.listener.current))
        return msg

    def CAN_subscriber(self, event):
        # Check if node is initialised
        if self.epsInitialised:
            # Check for node failure
            if any(self.ci.listener.activity[1:4]) == 0:
                rospy.loginfo("Error CAN Drive node died")
                rospy.loginfo(' '.join(map(str, self.ci.listener.activity[0])))
            # Publish electrical current message
            msg = self.get_eps_currents() # Current sensor data message
            self.eps_pub.publish(msg)
                    # Check for node activity
        else:
            rospy.loginfo("Initialise CAN EPS node")
            self.EPS_node_initialise()
            self.epsInitialised = True
        # Set node activity in listener
        self.ci.listener.activity[0] = 0

    def shutdown(self):
        # Cancel publisher
        self.timer.shutdown()
        # Cancel CAN listener
        self.ci.listener.stop()

"""
Main
"""
if __name__ == '__main__':
    # Start ROS node
    rospy.init_node("eps_control", anonymous=True)

    # Start ROS action
    server = LocomotionControl(rospy.get_name())
    rospy.spin()

rospy.on_shutdown(server.shutdown)
