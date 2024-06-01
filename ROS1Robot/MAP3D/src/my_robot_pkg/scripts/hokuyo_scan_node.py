#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
import serial
import math

class HokuyoNode:
    def __init__(self):
        rospy.init_node('hokuyo_node', anonymous=True)

        # Read parameters from the launch file
        self.serial_port = rospy.get_param('~serial_port', '/dev/ttyACM0')
        self.baud_rate = rospy.get_param('~baud_rate', 115200)
        self.frame_id = rospy.get_param('~frame_id', 'laser')

        # Setup serial connection
        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            rospy.loginfo(f"Connected to Hokuyo LiDAR on {self.serial_port} at {self.baud_rate} baud.")
        except serial.SerialException as e:
            rospy.logerr(f"Failed to connect to Hokuyo LiDAR: {e}")
            rospy.signal_shutdown("Failed to connect to LiDAR")
            return

        self.pub = rospy.Publisher('hokuyo_scan', LaserScan, queue_size=10)

        self.scan()

    def scan(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            rospy.loginfo("Requesting data from Hokuyo LiDAR...")
            self.ser.write(b'GD0000108000\n')
            data = self.ser.readline().strip()
            rospy.loginfo(f"Raw data received: {data}")

            if len(data) > 0 and data[0] == ord('@'):
                rospy.loginfo("Valid data received, processing...")
                scan_msg = LaserScan()
                scan_msg.header.stamp = rospy.Time.now()
                scan_msg.header.frame_id = self.frame_id

                scan_msg.angle_min = -math.pi / 2
                scan_msg.angle_max = math.pi / 2
                scan_msg.angle_increment = (scan_msg.angle_max - scan_msg.angle_min) / 682.0
                scan_msg.time_increment = 0.0
                scan_msg.scan_time = 0.1
                scan_msg.range_min = 0.1
                scan_msg.range_max = 10.0

                ranges = []
                for i in range(44, len(data), 2):
                    range_data = data[i:i+2]
                    if range_data:
                        ranges.append(int(range_data, 16) / 1000.0)

                scan_msg.ranges = ranges
                self.pub.publish(scan_msg)
                rospy.loginfo("LaserScan message published")

if __name__ == '__main__':
    try:
        node = HokuyoNode()
    except rospy.ROSInterruptException:
        pass
