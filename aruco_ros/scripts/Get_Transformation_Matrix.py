import rospy
import tf2_ros
import numpy as np
from geometry_msgs.msg import TransformStamped
import tf.transformations
import cv2


def get_transform_matrix(base_frame, end_effector_frame):
    rospy.init_node('get_transform', anonymous=True)
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

def calibrate_eye_hand(R_gripper2base, t_gripper2base, R_target2cam, t_target2cam, eye_to_hand=True):

    if eye_to_hand:
        # change coordinates from gripper2base to base2gripper
        R_base2gripper, t_base2gripper = [], []
        for R, t in zip(R_gripper2base, t_gripper2base):
            R_b2g = R.T
            t_b2g = -R_b2g @ t
            R_base2gripper.append(R_b2g)
            t_base2gripper.append(t_b2g)
        
        # change parameters values
        R_gripper2base = R_base2gripper
        t_gripper2base = t_base2gripper

    # calibrate
    R, t = cv2.calibrateHandEye(
        R_gripper2base=R_gripper2base,
        t_gripper2base=t_gripper2base,
        R_target2cam=R_target2cam,
        t_target2cam=t_target2cam,
    )

    return R, t

if __name__ == '__main__':
    base_frame = "uwarl_base_link"
    end_effector_frame = "front_right_wheel_link"
    transform_matrix, rotation_matrix = get_transform_matrix(base_frame, end_effector_frame)
    print("Transformation Matrix: \n", transform_matrix)
    print("Rotation Matrix: \n", rotation_matrix)
    t_gripper2base = transform_matrix[:3, -1].reshape(3, 1)
    print(t_gripper2base)
    R_gripper2base = rotation_matrix



# Example: Three sets of rotations and translations
R_gripper2base = [np.array([[1, 0, 0],
                             [0, 3, 0],
                             [0, 0, 1]]),
                  np.array([[0, -1, 0],
                            [1, 0, 0],
                            [0, 0, 1]]),
                  np.array([[0, 1, 0],
                            [-1, 0, 0],
                            [0, 0, 1]])]  # Add at least 3 sets

t_gripper2base = [np.array([20, 0, 1]).reshape(3, 1),
                  np.array([10, 0, 1]).reshape(3, 1),
                  np.array([30, 0, 2]).reshape(3, 1)]  # Add at least 3 sets

R_target2cam = [np.array([[1, 0, 0],
                          [0, 1, 0],
                          [0, 0, 1]]),
                np.array([[0, -1, 0],
                          [1, 0, 0],
                          [0, 0, 1]]),
                np.array([[0, 1, 0],
                          [-1, 0, 0],
                          [0, 0, 1]])]  # Add at least 3 sets

t_target2cam = [np.array([4, 0, 1]).reshape(3, 1),
                np.array([3, 0, 1]).reshape(3, 1),
                np.array([5, 0, 1]).reshape(3, 1)]  # Add at least 3 sets
    
R, t = cv2.calibrateHandEye(
        R_gripper2base=R_gripper2base,
        t_gripper2base=t_gripper2base,
        R_target2cam=R_target2cam,
        t_target2cam=t_target2cam,
    )
print(R)
print(t)