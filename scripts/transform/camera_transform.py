#!/usr/bin/env python  
import rospy
import tf
import tf2_ros
import geometry_msgs.msg
from maniros.msg import CameraHeadPose
from math import sin, cos, radians

def handle_camera_pose(msg, camera_name):

    # Generate transformation message
    br = tf2_ros.TransformBroadcaster()
    tf = geometry_msgs.msg.TransformStamped()

    tf.header.stamp = rospy.Time.now()
    tf.header.frame_id = "base_link"
    tf.child_frame_id = camera_name

    # Calculate position according to camera head orientation
    a = sin(radians(msg.pitch)) * cameraO

    # Assign transformation
    # Translation
    tf.transform.translation.x = camera_headX + sin(radians(msg.yaw)) * a
    tf.transform.translation.y = camera_headY - cos(radians(msg.yaw)) * a
    tf.transform.translation.z = camera_headZ + cos(radians(msg.pitch)) * cameraO
    # Quaternion
    q = tf.transformations.quaternion_from_euler(msg.yaw, msg.pitch, 0)
    tf.transform.rotation.x = q[0]
    tf.transform.rotation.y = q[1]
    tf.transform.rotation.z = q[2]
    tf.transform.rotation.w = q[3]

    # Broadcast
    br.sendTransform(tf)

if __name__ == '__main__':
    rospy.init_node('camera_tf_broadcaster')
    # Get camera parameters
    camera_name = rospy.get_param('~camera')
    camera_headX = rospy.get_param('camera_headX')
    camera_headY = rospy.get_param('camera_headY')
    camera_headZ = rospy.get_param('camera_headZ')
    cameraO = rospy.get_param('%sO' % camera_name)
    rospy.Subscriber('/%s/pose' % camera_name,
                     CameraHeadPose,
                     handle_camera_pose,
                     camera_name)
    rospy.spin()
