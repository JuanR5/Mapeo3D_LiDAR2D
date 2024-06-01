#!/usr/bin/env python3
# ROS2 libraries
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float64MultiArray, Float32
from sensor_msgs.msg import JointState
# Python libraries
import math

class JointGroupPositionPublisher(Node):
    def __init__(self):
        super().__init__('joint_group_position_publisher')
        
        self.joint_state_msg = JointState()

        self.current_position_in_degrees = 0.0
        self.current_position_in_radians = math.radians(self.current_position_in_degrees)
        self.declare_parameter("max_rotation_angle_in_degrees", 35)
        self.final_position_in_degrees = self.get_parameter("max_rotation_angle_in_degrees").get_parameter_value().integer_value
        self.final_position_in_radians = math.radians(self.final_position_in_degrees)
        
        self.joint_group_position_pub = self.create_publisher(
             Float64MultiArray, 
             "/joint_group_position_controller/commands", 
             10
        )

        self.angle_in_degrees_publisher = self.create_publisher(
            Int32,
            "/vonns2/serial_port/angle_in_degrees",
            10
        )
        
        self.angle_in_radians_publisher = self.create_publisher(
            Int32,
            "/vonns2/serial_port/angle_in_radians",
            10
        )
        
        self.rviz_publisher = self.create_publisher(
            JointState,
            "/rviz_angle",
            10
        )
        
        self.angle_subscriber = self.create_subscription(
            Float32,
            'motor_angle',
            self.angle_callback,
            10
        )

        self.angle_in_degrees_msg = Int32()
        self.angle_in_radians_msg = Int32()

        self.joint_group_position_msg = Float64MultiArray()
        self.joint_group_position_timer = self.create_timer(1.0, self.publish_position_in_joints)

    def angle_callback(self, msg):
        self.current_position_in_degrees = msg.data
        print(f"servo_joint: {self.current_position_in_degrees} °")
        
        angle = math.radians(self.current_position_in_degrees)
        self.joint_state_msg.name = ["servo_joint"]
        self.joint_state_msg.position = [angle]
        self.rviz_publisher.publish(self.joint_state_msg)

    def publish_position_in_joints(self):
        self.joint_group_position_msg.data = [self.current_position_in_radians]
        self.joint_group_position_pub.publish(self.joint_group_position_msg)
        
        self.angle_in_degrees_msg.data = int(self.current_position_in_degrees)
        self.angle_in_degrees_publisher.publish(self.angle_in_degrees_msg)
        
        self.angle_in_radians_msg.data = int(self.current_position_in_radians)
        self.angle_in_radians_publisher.publish(self.angle_in_radians_msg)

        self.get_logger().info(f"Position in Joint Group: servo_joint: {self.current_position_in_degrees} °")

def main(args=None):
    rclpy.init(args=args)
    joint_group_position_pub_node = JointGroupPositionPublisher()
    try:
        rclpy.spin(joint_group_position_pub_node)
    except KeyboardInterrupt:
        joint_group_position_pub_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

