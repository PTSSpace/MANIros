#!/usr/bin/env python  
import rospy
import tf
import tf2_ros
import geometry_msgs.msg

if __name__ == '__main__':
    rospy.init_node('imu_tf_broadcaster')
    # Get imu parameters
    imu_name = rospy.get_param('~imu')
    imuX = rospy.get_param('%sX' % imu_name)
    imuY = rospy.get_param('%sY' % imu_name)
    imuZ = rospy.get_param('%sZ' % imu_name)

    # Generate transformation message
    br = tf2_ros.StaticTransformBroadcaster()
    tf = geometry_msgs.msg.TransformStamped()

    tf.header.stamp = rospy.Time.now()
    tf.header.frame_id = "base_link"
    tf.child_frame_id = imu_name

    # Assign transformation
    # Translation
    tf.transform.translation.x = imuX
    tf.transform.translation.y = imuY
    tf.transform.translation.z = imuZ
    # Quaternion
    tf.transform.rotation.x = 0.0
    tf.transform.rotation.y = 0.0
    tf.transform.rotation.z = 0.0
    tf.transform.rotation.w = 1.0

    # Broadcast
    br.sendTransform(tf)
    rospy.spin()
