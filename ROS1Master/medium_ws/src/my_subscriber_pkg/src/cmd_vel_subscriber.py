#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

def callback(msg):
    rospy.loginfo("Received cmd_vel message: %s", msg)

def subscriber():
    rospy.init_node('cmd_vel_subscriber', anonymous=True)
    rospy.Subscriber('/cmd_vel', Twist, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        subscriber()
    except rospy.ROSInterruptException:
        pass
