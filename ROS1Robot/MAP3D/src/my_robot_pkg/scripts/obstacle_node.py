#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import tf
import numpy as np
import math

class GoToGoal:
    def __init__(self):
        rospy.init_node('go_to_goal', anonymous=True)
        
        self.goal_x = float(input("Enter the goal x position: "))
        self.goal_y = float(input("Enter the goal y position: "))
        # self.goal_orientation = np.deg2rad(float(input("Enter the desired orientation at the goal (in Degrees): ")))
        # self.start_orientation = np.deg2rad(float(input("Enter the starting orientation (in Degrees): ")))

        self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_callback)
        self.scan_sub = rospy.Subscriber('scan', LaserScan, self.scan_callback)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_orientation = 0.0
        self.scan_ranges = []
        
        self.xi = 0
        self.yi = 0

        self.odom_received = False 
        self.moving_to_goal = False
        self.obstacle_detected = False
        self.stop_robot()
        
        self.timer = rospy.Timer(rospy.Duration(0.1), self.move)

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        _, _, self.current_orientation = self.quaternion_to_euler(orientation_q)
        self.odom_received = True

    def scan_callback(self, msg):
        self.scan_ranges = msg.ranges
        self.obstacle_detected = False
        for i in range(460):
            if 0 < self.scan_ranges[i] < 0.4:
                self.obstacle_detected = True
                
                self.stop_robot()
                
                self.move_angle()
                
                self.move_to_goal()
                
                print("ObstacleD")
                break

    def move(self, event):
        if not self.odom_received:
            self.stop_robot()
            rospy.loginfo('Waiting for odometry data...')
            return
        if not self.scan_ranges:
            self.stop_robot()
            rospy.loginfo('Waiting for scan data...')
            return
        
        if self.obstacle_detected:
            self.stop_robot()
            self.move_angle()
        
        if not self.moving_to_goal:
            self.move_angle()
        else:
            self.move_to_goal()

        if self.timer_periodic_check():
            self.moving_to_goal = False

    def move_angle(self):
        obstacle_detected = False
        theta = 0
        self.xi = 0
        self.yi = 0
        angular_velocity = 0.0
        
        for i in range(115):
            if 0 < self.scan_ranges[i] < 0.4:
                obstacle_detected = True
                self.xi += self.scan_ranges[i] * np.cos((2*np.pi*i)/460)
                self.yi += self.scan_ranges[i] * np.sin((2*np.pi*i)/460)
                theta = np.arctan2(self.yi, self.xi) - np.pi/2

        for i in range(345, 460):
            if 0 < self.scan_ranges[i] < 0.4:
                obstacle_detected = True
                if np.abs(self.scan_ranges[i]) < np.abs(self.xi) and np.abs(self.scan_ranges[i]) < np.abs(self.yi):
                    theta = np.arctan2(self.scan_ranges[i] * np.sin((2*np.pi*i)/460), 
                                       self.scan_ranges[i] * np.cos((2*np.pi*i)/460)) - np.pi/2

        if obstacle_detected:
            if theta > 0:
                target_angle = theta + np.pi / 3
            else:    
                target_angle = theta - np.pi / 3
            
            angle_error = target_angle - self.current_orientation
            angle_error = np.arctan2(np.sin(angle_error), np.cos(angle_error))
            #linear_velocity_x = 0.08

            if abs(angle_error) > 0.1:
                angular_velocity = 0.3 if angle_error > 0 else -0.3
            elif abs(angle_error) > 0.06:
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
            
            #linear_velocity_x = 0.0

        cmd_vel_msg = Twist()
        #cmd_vel_msg.linear.x = linear_velocity_x
        cmd_vel_msg.angular.z = angular_velocity
        self.cmd_vel_pub.publish(cmd_vel_msg)

    def move_to_goal(self):
        distance_to_goal = np.sqrt((self.goal_x - self.current_x)**2 + (self.goal_y - self.current_y)**2)
        rospy.loginfo(f"Distance to goal: {distance_to_goal}")

        if distance_to_goal > 0.15:
            linear_velocity_x = 0.15
        elif distance_to_goal > 0.05:
            linear_velocity_x = 0.06
        else:
            self.stop_robot()
            self.moving_to_goal = False
            rospy.loginfo('Goal reached!')
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
        euler = tf.transformations.euler_from_quaternion(quaternion)
        return euler
    
    def timer_periodic_check(self):
        current_time = rospy.Time.now().to_nsec()
        if hasattr(self, 'last_check_time'):
            if current_time - self.last_check_time > 3e9:  # 3 seconds
                self.last_check_time = current_time
                return True
        else:
            self.last_check_time = current_time
        return False

if __name__ == '__main__':
    try:
        GoToGoal()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
