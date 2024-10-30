import rospy
import tf2_ros
import numpy as np
from geometry_msgs.msg import TransformStamped
import tf.transformations
import cv2

# Initialize storage for transformations
R_gripper2base = []
t_gripper2base = []
R_target2cam = []
t_target2cam = []

def get_transform_matrix(base_frame, end_effector_frame):
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    try:
        # Wait for the transformation to be available
        trans = tfBuffer.lookup_transform(base_frame, end_effector_frame, rospy.Time(0), rospy.Duration(5.0))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        rospy.logerr('Transform not available')
        return None

    # Extract translation and rotation
    translation = [trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z]
    rotation = [trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w]

    # Convert quaternion to a rotation matrix
    rotation_matrix = tf.transformations.quaternion_matrix(rotation)[:3, :3]

    # Create the transformation matrix (4x4)
    transform_matrix = np.eye(4)
    transform_matrix[:3, :3] = rotation_matrix
    transform_matrix[:3, 3] = translation

    return transform_matrix, rotation_matrix

def transform_callback(msg):
    # Collect R_target2cam and t_target2cam from the message
    translation = [msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z]
    rotation = [msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w]
    
    rotation_matrix = tf.transformations.quaternion_matrix(rotation)[:3, :3]
    
    # Append R_target2cam and t_target2cam
    R_target2cam.append(rotation_matrix)
    t_target2cam.append(np.array(translation).reshape(3, 1))
    
    # Collect R_gripper2base and t_gripper2base using get_transform_matrix
    base_frame = "uwarl_base_link"
    end_effector_frame = "front_right_wheel_link"
    transform_matrix, rotation_matrix_gripper = get_transform_matrix(base_frame, end_effector_frame)
    
    if transform_matrix is not None:
        # Append R_gripper2base and t_gripper2base
        R_gripper2base.append(rotation_matrix_gripper)
        t_gripper2base.append(transform_matrix[:3, -1].reshape(3, 1))

def calibrate_eye_hand(publisher):
    # Perform the calibration
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
    publisher.publish(transform_msg)
    rospy.loginfo("Published Hand-Eye Calibration Matrix")

if __name__ == '__main__':
    rospy.init_node('calibration_node', anonymous=True)
    
    # Publisher for the hand-eye calibration matrix
    calibration_publisher = rospy.Publisher('/hand_eye_calibration_matrix', TransformStamped, queue_size=10)
    
    # Subscribe to the /arcuco_single/transform topic
    rospy.Subscriber('/arcuco_single/transform', TransformStamped, transform_callback)
    
    # Wait for a certain number of transformations to be collected
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        if len(R_gripper2base) >= 3:  # Wait until at least 3 data points are collected
            calibrate_eye_hand(calibration_publisher)
            break
        rate.sleep()
    
    rospy.spin()
