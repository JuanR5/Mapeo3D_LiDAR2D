#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import tf_transformations
import numpy as np

class GoToGoal(Node):
    def __init__(self):
        super().__init__('go_to_goal')
        self.goal_x = float(input("Enter the goal x position: "))
        self.goal_y = float(input("Enter the goal y position: "))
        self.goal_orientation = np.deg2rad(float(input("Enter the desired orientation at the goal (in Degrees): ")))
        
        self.start_orientation = np.deg2rad(float(input("Enter the starting orientation (in Degrees): ")))

        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            QoSProfile(depth=10))

        self.cmd_vel_pub = self.create_publisher(
            Twist,
            'cmd_vel',
            QoSProfile(depth=10))

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_orientation = 0.0

        self.moving_to_goal = False
        
        self.timer = self.create_timer(0.1, self.move)

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        _, _, self.current_orientation = self.quaternion_to_euler(orientation_q)
        # self.get_logger().info(f'Odometry - Position: [{self.current_x}, {self.current_y}, {np.rad2deg(self.current_orientation)}]')

    def move(self):
        if not self.moving_to_goal:
            self.move_angle()
        else:
            self.move_to_goal()
        
                # Periodically recheck the angle
        if self.timer_periodic_check():
            self.moving_to_goal = False

    def move_angle(self):
        angle_to_goal = np.arctan2(self.goal_y - self.current_y, self.goal_x - self.current_x)
        angle_error = angle_to_goal - self.current_orientation
        angle_error = np.arctan2(np.sin(angle_error), np.cos(angle_error))
        print("angle: ", np.rad2deg(angle_error))
        
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
        elif distance_to_goal > 0.05:
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
            if current_time - self.last_check_time > 2e9:  # 5 seconds
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
