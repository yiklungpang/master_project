#!/usr/bin/env python
import rospy
import tf
import tf2_ros
import tf2_msgs.msg
import geometry_msgs.msg


class AddCameraLink:
    """
        Class allowing to add the frame corresponding to the virtual kinect to the tf tree
    """
    def __init__(self):
        # Since the kinect is static, creating a static transform broadcaster
        self.broadcaster = tf2_ros.StaticTransformBroadcaster()
        # Initializing the message containing the transform
        static_camera_transform_stamped = geometry_msgs.msg.TransformStamped()
        # The link will be created from world to the kinect (frame defined as camera_link in the sdf file)
        static_camera_transform_stamped.header.frame_id = "world"
        static_camera_transform_stamped.header.stamp = rospy.Time.now()
        static_camera_transform_stamped.child_frame_id = "camera_link"
        # The translation corresponds to the offset between world (0,0,0) and the position of the kinect in the world
        static_camera_transform_stamped.transform.translation.x = 0
        static_camera_transform_stamped.transform.translation.y = 0.161
        static_camera_transform_stamped.transform.translation.z = 1.6565
        # Since in the world file the kinect is oriented, we should also operate
        # here a tranform so that the position of the point cloud in Rviz matches the reality.
        static_camera_transform_stamped.transform.rotation.x = 0.9816272
        static_camera_transform_stamped.transform.rotation.y = 0
        static_camera_transform_stamped.transform.rotation.z = 0
        static_camera_transform_stamped.transform.rotation.w = -0.19080

        self.broadcaster.sendTransform([static_camera_transform_stamped])

if __name__ == '__main__':
    rospy.init_node('fixed_tf2_broadcaster')
    tfb = AddCameraLink()
    rospy.spin()
