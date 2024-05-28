#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import tf_transformations
import numpy as np
import math
import time

class GoToGoal(Node):
    def __init__(self):
        super().__init__('go_to_goal')
        self.goal_x = float(input("Enter the goal x position: "))
        self.goal_y = float(input("Enter the goal y position: "))
        #self.goal_orientation = np.deg2rad(float(input("Enter the desired orientation at the goal (in Degrees): ")))
        #self.start_orientation = np.deg2rad(float(input("Enter the starting orientation (in Degrees): ")))

        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            QoSProfile(depth=10))

        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            QoSProfile(depth=10))

        self.cmd_vel_pub = self.create_publisher(
            Twist,
            'cmd_vel',
            QoSProfile(depth=10))

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_orientation = 0.0
        self.scan_ranges = []
        self.leng = 0
        
        self.odom_received = False 
        self.moving_to_goal = False
        self.obstacle_detected = False
        self.stop_robot()
        
        self.timer = self.create_timer(0.1, self.move)
        

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        _, _, self.current_orientation = self.quaternion_to_euler(orientation_q)
        self.odom_received = True
        # self.get_logger().info(f'Odometry - Position: [{self.current_x}, {self.current_y}, {np.rad2deg(self.current_orientation)}]')

    def scan_callback(self, msg):
        self.scan_ranges = msg.ranges
        self.obstacle_detected = False
        self.leng = len(self.scan_ranges)
        for i in range(self.leng):
            if 0 < self.scan_ranges[i] < 0.4:
                self.obstacle_detected = True
                self.stop_robot()
                self.move_angle()
                self.move_to_goal()
                break
        

    def move(self):
        ## Check if the data is being recived correctly
        if not self.odom_received:
            self.stop_robot()
            self.get_logger().info('Waiting for odometry data...')
            return
        if not self.scan_ranges:
            self.stop_robot()
            self.get_logger().info('Waiting for scan data...')
            return
        
        if self.obstacle_detected:
            self.stop_robot()
            self.move_angle()
        elif not self.moving_to_goal:
            self.move_angle()
        else:
            self.move_to_goal()

        # Periodically recheck the angle
        if self.timer_periodic_check():
            self.moving_to_goal = False

    def move_angle(self):

        # Check for obstacles in front left and front right
        obstacle_detected = False
        xi_left = 0.0
        yi_left = 0.0
        xi_right = 0.0
        yi_right = 0.0
        angular_velocity = 0.0
        
        for i in range(int(self.leng/4)):
            if 0 < self.scan_ranges[i] < 0.4:
                obstacle_detected = True
                xi_left += self.scan_ranges[i] * np.cos((2*np.pi*i)/self.leng)
                yi_left += self.scan_ranges[i] * np.sin((2*np.pi*i)/self.leng)
        theta_left = np.arctan2(yi_left,xi_left)
        #if theta_left is not None:
            #print("leftObs: ", theta_left)
            

        for i in range(int(self.leng*3/4), self.leng):
            if 0 < self.scan_ranges[i] < 0.4:
                obstacle_detected = True
                xi_right += self.scan_ranges[i] * np.cos((2*np.pi*i)/self.leng)
                yi_right += self.scan_ranges[i] * np.sin((2*np.pi*i)/self.leng)            
        theta_right = np.arctan2(yi_right,xi_right)
        #if theta_right is not None:
            #print("RightObs: ", theta_right)

        if obstacle_detected:
                # Combine the angles from both sides if obstacles are detected on both sides
            theta = (theta_left + theta_right)/2
            print("Theta: ", theta)
            if theta > 0:
                target_angle = theta - np.pi / 3
            elif theta < 0:    
                target_angle = theta + np.pi / 3  # Rotate away from the obstacle
            else:
                target_angle = np.pi/2  # Default value in case no obstacles are detected
            
            angle_error = target_angle - self.current_orientation
            angle_error = np.arctan2(np.sin(angle_error), np.cos(angle_error))
            print("Angle Error: ", angle_error)

            if abs(angle_error) > 0.08:
                angular_velocity = 0.2 if angle_error > 0 else -0.2
            elif abs(angle_error) > 0.05:
                angular_velocity = 0.08 if angle_error > 0 else -0.08
            else:
                angular_velocity = 0.0
                self.moving_to_goal = True

        else:
            angle_to_goal = np.arctan2(self.goal_y - self.current_y, self.goal_x - self.current_x)
            angle_error = angle_to_goal - self.current_orientation
            angle_error = np.arctan2(np.sin(angle_error), np.cos(angle_error))
           
            if abs(angle_error) > 0.1:
                angular_velocity = 0.1 if angle_error > 0 else -0.1
            elif abs(angle_error) > 0.08:
                angular_velocity = 0.08 if angle_error > 0 else -0.08
            else:
                angular_velocity = 0.0
                self.moving_to_goal = True

        cmd_vel_msg = Twist()
        cmd_vel_msg.angular.z = angular_velocity
        self.cmd_vel_pub.publish(cmd_vel_msg)

    def move_to_goal(self):
                
        distance_to_goal = np.sqrt((self.goal_x - self.current_x)**2 + (self.goal_y - self.current_y)**2)
        print("distance: ", distance_to_goal)

        if distance_to_goal > 0.15:
            linear_velocity_x = 0.1
        elif distance_to_goal > 0.04:
            linear_velocity_x = 0.06
        else:
            self.stop_robot()
            self.moving_to_goal = False
            self.get_logger().info('Goal reached!')
            return

        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = linear_velocity_x
        self.cmd_vel_pub.publish(cmd_vel_msg)
        
    def stop_robot(self):
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = 0.0
        cmd_vel_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd_vel_msg)

    def quaternion_to_euler(self, orientation_q):
        quaternion = (
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w
        )
        euler = tf_transformations.euler_from_quaternion(quaternion)
        return euler
    
    def timer_periodic_check(self):
        # Return True periodically to recheck angle adjustment
        current_time = self.get_clock().now().nanoseconds
        if hasattr(self, 'last_check_time'):
            if current_time - self.last_check_time > 3e9:  # 3 seconds
                self.last_check_time = current_time
                return True
        else:
            self.last_check_time = current_time
        return False


def main(args=None):
    rclpy.init(args=args)
    node = GoToGoal()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
