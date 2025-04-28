#!/usr/bin/env python3

import rospy
from barrett_wam_msgs.msg import RTJointPos

def wam_joint_publisher():
    rospy.init_node('wam_joint_publisher', anonymous=True)
    pub = rospy.Publisher('/wam/jnt_pos_cmd', RTJointPos, queue_size=1)
    rate = rospy.Rate(50)  # 10 Hz
    msg = RTJointPos()
    
    
    msg.joints = [0.463, 0.766, -0.0681, 1.178, 0, 1.17, 0.402]  # Example joint positions
    msg.rate_limits = [0.1] * 7  # Example rate limits
    count  = 0
    
    while count < 200:
        
        
        rospy.loginfo("Publishing joint positions: %s", msg.joints)
        rospy.loginfo("Publishing rate limits: %s", msg.rate_limits)
        pub.publish(msg)
        count+=1
        rate.sleep()

if __name__ == '__main__':
    try:
        wam_joint_publisher()
    except rospy.ROSInterruptException:
        pass
