#!/usr/bin/env python3

import time
import numpy as np
import asyncio
import rclpy

from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from tf2_ros.transform_listener import TransformListenerException
from geometry_msgs.msg import PoseStamped, Point
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import NavigateToPoseAction, NavigateToPoseGoal
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile
from rclpy.time import Time
from rclpy.duration import Duration

class MapNavigation(Node):
    def __init__(self):
        super().__init__('map_navigation')
        self.goal_reached = False
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.qos_profile = QoSProfile(depth=10)
        self.navigate_client = ActionClient(self, NavigateToPose, 'navigate_to_pose', self.qos_profile)
        self.goal_sent = False

    async def move_to_goal(self, x_goal, y_goal, orientation_z, orientation_w):
        goal_msg = NavigateToPoseGoal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position = Point(x=x_goal, y=y_goal, z=0.0)
        goal_msg.pose.pose.orientation.z = orientation_z
        goal_msg.pose.pose.orientation.w = orientation_w
        await self.send_goal(goal_msg)

    async def send_goal(self, goal_msg):
        self.get_logger().info('Sending goal: {0}'.format(goal_msg.pose.pose.position))
        self.navigate_client.wait_for_server()
        self.goal_sent = True
        self.navigate_client.send_goal_async(goal_msg, self.done_callback)

    def done_callback(self, future):
        self.get_logger().info('Done callback: {0}'.format(future.result()))
        self.goal_reached = True

    async def spin(self):
        while rclpy.ok():
            await asyncio.sleep(0.1)

def main(args=None):
    rclpy.init(args=args)
    navigation = MapNavigation()
    asyncio.run(navigation.spin())
    navigation.shutdown()
    rclpy.shutdown()

if __name__ == '__main__':
    main()