#!/usr/bin/env python

import unittest
import rospy, rostest
import time
import tf2_ros


class ImuTransformTest(unittest.TestCase):
    def __init__(self, *args):
        super(ImuTransformTest, self).__init__(*args)
        rospy.init_node("test_input", anonymous=True)

        self.tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(self.tfBuffer)
        # Get imu parameters
        self.imu_name = rospy.get_param('~imu')
        self.success = False

    def test_basic_node_actions(self):
        timeout_t = time.time() + 10.0 #10 seconds
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown() and not self.success and time.time() < timeout_t:
            self.success = True
            try:
                data = self.tfBuffer.lookup_transform(self.imu_name, 'base_link', rospy.Time())
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                self.success = False
                rate.sleep()
                continue

        if self.success:
            rospy.loginfo("Translational transform: I've heard x:%d y:%d z:%d." % (
                data.transform.translation.x,
                data.transform.translation.y,
                data.transform.translation.z
            ))
            rospy.loginfo("Rotational transform: I've heard qx:%d qy:%d qz:%d qw:%d." % (
                data.transform.rotation.x,
                data.transform.rotation.y,
                data.transform.rotation.z,
                data.transform.rotation.w
            ))

        self.assertTrue(self.success, 'Callback has not been called')

if __name__ == '__main__':
    rostest.rosrun('maniros', 'test_imu_transform', ImuTransformTest)
