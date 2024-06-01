#!/usr/bin/env python

import rospy
import serial
import time
from std_msgs.msg import Float32

class SerialCommanderNode:
    def __init__(self):
        rospy.init_node('serial_commander', anonymous=True)
        self.serial_port = rospy.get_param('~serial_port', '/dev/ttyACM0')  # Replace with your serial port
        self.baud_rate = rospy.get_param('~baud_rate', 9600)
        self.ser = serial.Serial(self.serial_port, self.baud_rate)
        time.sleep(2)  # Wait for the serial connection to establish
        self.command_counter = 0
        self.angle = -25.0  # Start at -25°
        self.angle_pub = rospy.Publisher('motor_angle', Float32, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(0.5), self.send_serial_commands)  # Adjusted to 0.5 seconds for publishing

    def send_serial_commands(self, event):
        # Send the command to update angle 10 times, each time incrementing the angle by 5°
        for _ in range(10):
            self.ser.write(b'-142\n')
            self.angle += 5  # Increase angle by 5°
            
            time.sleep(1.2)  # Wait 1 second between commands
            self.publish_angle()
        
        # Reset the angle to -25° by sending the command 1420
        self.ser.write(b'1420\n')
        self.angle = -25.0  # Reset angle to -25°
        
        time.sleep(10)  # Wait 10 seconds before restarting the cycle
        self.publish_angle()

    def publish_angle(self):
        angle_msg = Float32()
        angle_msg.data = self.angle
        self.angle_pub.publish(angle_msg)

def main():
    node = SerialCommanderNode()
    rospy.spin()

if __name__ == '__main__':
    main()

