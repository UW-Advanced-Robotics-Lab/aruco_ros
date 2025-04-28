#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped

def cmd_vel_callback(msg):
    stamped_msg = TwistStamped()
    stamped_msg.header.stamp = rospy.Time.now()
    stamped_msg.header.frame_id = "base_link"  # Change if needed
    stamped_msg.twist = msg
    
    pub.publish(stamped_msg)

if __name__ == "__main__":
    rospy.init_node("cmd_vel_stamper")
    
    pub = rospy.Publisher("/cmd_vel_stamped", TwistStamped, queue_size=10)
    sub = rospy.Subscriber("/uwarl/robotnik_base_control/cmd_vel", Twist, cmd_vel_callback)
    
    rospy.spin()
