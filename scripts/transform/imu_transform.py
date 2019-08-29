#!/usr/bin/env python  
import rospy
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
    stfS = geometry_msgs.msg.TransformStamped()

    stfS.header.stamp = rospy.Time.now()
    stfS.header.frame_id = "base_link"
    stfS.child_frame_id = imu_name

    # Assign transformation
    # Translation
    stfS.transform.translation.x = imuX
    stfS.transform.translation.y = imuY
    stfS.transform.translation.z = imuZ
    # Quaternion
    stfS.transform.rotation.x = 0.0
    stfS.transform.rotation.y = 0.0
    stfS.transform.rotation.z = 0.0
    stfS.transform.rotation.w = 1.0

    # Broadcast
    br.sendTransform(stfS)
    rospy.spin()
