import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage

class MySubscriber(Node):

    def __init__(self):
        super().__init__('my_subscriber')
        
        # Subscription to Odometry topic
        self.odom_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
        
        # Subscription to LaserScan topic
        self.scan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10)
        
        # Subscription to TFMessage topic
        self.tf_subscription = self.create_subscription(
            TFMessage,
            'tf',
            self.tf_callback,
            10)
        
        self.odom_subscription  # prevent unused variable warning
        self.scan_subscription  # prevent unused variable warning
        self.tf_subscription  # prevent unused variable warning

    def odom_callback(self, msg):
        # Accessing specific fields of the Odometry message
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        self.get_logger().info(f'Odometry - Position: [{position.x}, {position.y}, {position.z}]')
        self.get_logger().info(f'Odometry - Orientation: [{orientation.x}, {orientation.y}, {orientation.z}, {orientation.w}]')

    def scan_callback(self, msg):
        # Process LaserScan message
        # You can access the data from the LaserScan message here
        pass

    def tf_callback(self, msg):
        # Process TFMessage
        for transform in msg.transforms:
            self.get_logger().info(f'TF - Transform: {transform}')

def main(args=None):
    rclpy.init(args=args)
    subscriber = MySubscriber()
    rclpy.spin(subscriber)
    subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
