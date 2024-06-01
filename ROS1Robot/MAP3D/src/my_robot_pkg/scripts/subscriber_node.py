#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan

def scan_callback(scan_data):
    # Print the length of the ranges array
    rospy.loginfo("Length of the scan ranges array: %d", len(scan_data.ranges))

def main():
    # Initialize the node
    rospy.init_node('scan_length_node', anonymous=True)

    # Subscribe to the /scan topic
    rospy.Subscriber('/scan', LaserScan, scan_callback)

    # Keep the node running
    rospy.spin()

if __name__ == '__main__':
    main()
