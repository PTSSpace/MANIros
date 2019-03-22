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
    tfS = geometry_msgs.msg.TransformStamped()

    tfS.header.stamp = rospy.Time.now()
    tfS.header.frame_id = "base_link"
    tfS.child_frame_id = camera_name

    # Calculate position according to camera head orientation
    a = sin(radians(msg.pitch)) * cameraO

    # Assign transformation
    # Translation
    tfS.transform.translation.x = camera_headX + sin(radians(msg.yaw)) * a
    tfS.transform.translation.y = camera_headY - cos(radians(msg.yaw)) * a
    tfS.transform.translation.z = camera_headZ + cos(radians(msg.pitch)) * cameraO
    # Quaternion
    q = tf.transformations.quaternion_from_euler(msg.yaw, msg.pitch, 0)
    tfS.transform.rotation.x = q[0]
    tfS.transform.rotation.y = q[1]
    tfS.transform.rotation.z = q[2]
    tfS.transform.rotation.w = q[3]

    # Broadcast
    br.sendTransform(tfS)

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
