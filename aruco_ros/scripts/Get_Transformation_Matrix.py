import rospy
import tf2_ros
import numpy as np
from geometry_msgs.msg import TransformStamped

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

if __name__ == '__main__':
    base_frame = "base_link"
    end_effector_frame = "end_effector_link"
    transform_matrix, rotation_matrix = get_transform_matrix(base_frame, end_effector_frame)
    print("Transformation Matrix: \n", transform_matrix)
    print("Rotation Matrix: \n", rotation_matrix)
