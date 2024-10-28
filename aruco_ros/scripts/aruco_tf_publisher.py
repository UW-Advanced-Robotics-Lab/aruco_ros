#!/usr/bin/env python3
import rospy
import cv2
import cv2.aruco as aruco
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import tf2_ros
from geometry_msgs.msg import TransformStamped

class ArucoTfPublisher:
    def __init__(self):
        # Initialize the node
        rospy.init_node('aruco_tf_publisher', anonymous=True)

        # Camera image and camera info topics
        self.image_topic = rospy.get_param('~image_topic', '/camera/color/image_raw')
        self.camera_info_topic = rospy.get_param('~camera_info_topic', '/camera/color/camera_info')

        # Create CvBridge for image conversion
        self.bridge = CvBridge()

        # ArUco marker detection parameters
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        self.parameters = aruco.DetectorParameters_create()

        # To store camera matrix and distortion coefficients
        self.camera_matrix = None
        self.dist_coeffs = None

        # Transformation broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        # Subscribe to the image topic
        self.image_sub = rospy.Subscriber(self.image_topic, Image, self.image_callback)

        # Subscribe to the camera info topic to get camera intrinsics
        self.camera_info_sub = rospy.Subscriber(self.camera_info_topic, CameraInfo, self.camera_info_callback)

    def camera_info_callback(self, msg):
        # Extract the camera matrix and distortion coefficients from CameraInfo message
        self.camera_matrix = np.array(msg.K).reshape(3, 3)
        self.dist_coeffs = np.array(msg.D)

        rospy.loginfo("Camera matrix and distortion coefficients received")

    def image_callback(self, msg):
        # Ensure we have the camera matrix and distortion coefficients
        if self.camera_matrix is None or self.dist_coeffs is None:
            rospy.logwarn("Waiting for camera info")
            return

        # Convert ROS Image to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # Detect markers in the image
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)

        if ids is not None:
            # Estimate pose of each marker
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, 0.05, self.camera_matrix, self.dist_coeffs)

            # Assume we only care about the first detected marker for simplicity
            rvec, tvec = rvecs[0], tvecs[0]

            # Compute the rotation matrix from the rotation vector
            rotation_matrix, _ = cv2.Rodrigues(rvec)

            # Create a TransformStamped message
            transform_msg = TransformStamped()
            transform_msg.header.stamp = rospy.Time.now()
            transform_msg.header.frame_id = "camera_frame"
            transform_msg.child_frame_id = "aruco_board"

            # Set translation (from tvec)
            transform_msg.transform.translation.x = tvec[0][0]
            transform_msg.transform.translation.y = tvec[0][1]
            transform_msg.transform.translation.z = tvec[0][2]

            # Convert rotation matrix to quaternion
            rotation_quat = tf2_ros.transformations.quaternion_from_matrix(np.vstack((rotation_matrix, [0, 0, 0, 1])))

            transform_msg.transform.rotation.x = rotation_quat[0]
            transform_msg.transform.rotation.y = rotation_quat[1]
            transform_msg.transform.rotation.z = rotation_quat[2]
            transform_msg.transform.rotation.w = rotation_quat[3]

            # Publish the transform
            self.tf_broadcaster.sendTransform(transform_msg)

            # Optionally, print or log the transformation and rotation matrices
            rospy.loginfo(f"Translation: {tvec}")
            rospy.loginfo(f"Rotation Matrix:\n{rotation_matrix}")

if __name__ == '__main__':
    try:
        node = ArucoTfPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
