import rospy
import tf2_ros
import numpy as np
from geometry_msgs.msg import TransformStamped, PoseStamped
import tf.transformations
import cv2

# Initialize storage for transformations
R_gripper2base = []
t_gripper2base = []
R_target2cam = []
t_target2cam = []

class CalibrationNode:
    def __init__(self):
        # Cache for the latest WAM pose
        self.latest_wam_pose = None
        
        # Initialize node
        rospy.init_node('calibration_node', anonymous=True)
        
        # Publisher for the hand-eye calibration matrix
        self.calibration_publisher = rospy.Publisher('/hand_eye_calibration_matrix', TransformStamped, queue_size=10)
        
        # Subscriber to the /arcuco_single/transform topic for camera-to-target transformations
        rospy.Subscriber('/arcuco_single/transform', TransformStamped, self.transform_callback)
        
        # Subscriber to the /wam/pose topic to update the latest WAM pose
        rospy.Subscriber('/wam/pose', PoseStamped, self.wam_pose_callback)
        
    def wam_pose_callback(self, msg):
        """Update the latest WAM pose."""
        self.latest_wam_pose = msg

    def transform_callback(self, msg):
        """Process a new ArUco transform message along with the latest WAM pose."""
        if self.latest_wam_pose is None:
            rospy.logwarn("No WAM pose received yet.")
            return
        
        # Extract and store ArUco transform (R_target2cam, t_target2cam)
        aruco_translation = [msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z]
        aruco_rotation = [msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w]
        aruco_rotation_matrix = tf.transformations.quaternion_matrix(aruco_rotation)[:3, :3]
        
        R_target2cam.append(aruco_rotation_matrix)
        t_target2cam.append(np.array(aruco_translation).reshape(3, 1))

        # Extract and store the latest WAM pose (R_gripper2base, t_gripper2base)
        wam_translation = [self.latest_wam_pose.pose.position.x, self.latest_wam_pose.pose.position.y, self.latest_wam_pose.pose.position.z]
        wam_rotation = [self.latest_wam_pose.pose.orientation.x, self.latest_wam_pose.pose.orientation.y, self.latest_wam_pose.pose.orientation.z, self.latest_wam_pose.pose.orientation.w]
        wam_rotation_matrix = tf.transformations.quaternion_matrix(wam_rotation)[:3, :3]
        
        R_gripper2base.append(wam_rotation_matrix)
        t_gripper2base.append(np.array(wam_translation).reshape(3, 1))
        
        rospy.loginfo("Synchronized ArUco transform and latest WAM pose recorded.")
        
        # Check if enough data has been collected for calibration
        if len(R_gripper2base) >= 3:
            self.calibrate_eye_hand()

    def calibrate_eye_hand(self):
        """Perform eye-hand calibration and publish the result."""
        # Perform calibration
        R, t = cv2.calibrateHandEye(
            R_gripper2base=R_gripper2base,
            t_gripper2base=t_gripper2base,
            R_target2cam=R_target2cam,
            t_target2cam=t_target2cam,
        )

        # Prepare and publish the calibration transform as a TransformStamped message
        transform_msg = TransformStamped()
        transform_msg.header.stamp = rospy.Time.now()
        transform_msg.header.frame_id = "camera_optical_frame"
        transform_msg.child_frame_id = "hand_eye_calibration_frame"
        
        # Convert rotation matrix to quaternion
        quat = tf.transformations.quaternion_from_matrix(np.vstack((np.hstack((R, [[0], [0], [0]])), [[0, 0, 0, 1]])))
        transform_msg.transform.rotation.x = quat[0]
        transform_msg.transform.rotation.y = quat[1]
        transform_msg.transform.rotation.z = quat[2]
        transform_msg.transform.rotation.w = quat[3]

        # Set translation
        transform_msg.transform.translation.x = t[0]
        transform_msg.transform.translation.y = t[1]
        transform_msg.transform.translation.z = t[2]
        
        # Publish the message
        self.calibration_publisher.publish(transform_msg)
        rospy.loginfo("Published Hand-Eye Calibration Matrix")

if __name__ == '__main__':
    CalibrationNode()
    rospy.spin()
