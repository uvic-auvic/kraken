#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class DepthProcessor(Node):
    def __init__(self):
        super().__init__('depth_listener')
        self.subscription = self.create_subscription(
            LaserScan,
            '/submarine/depth',
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(LaserScan, '/submarine/depth_meters', 10)

    def listener_callback(self, msg):
        # Extract depth from the LaserScan message
        depth = msg.ranges[0]  # Use the first range value
        # Format message for clear output; assume sensor reading indicates distance below sea level
        self.get_logger().info(f'Submarine is {depth:.2f} m below sea level')
        x = f'Submarine is {depth:.2f} m below sea level'
        # Publish the depth value
        depth_msg = LaserScan()
        depth_msg.header = msg.header
        depth_msg.ranges = x
        ##depth_msg.ranges = [depth]
        self.publisher.publish(x)

def main(args=None):
    rclpy.init(args=args)
    depth_processor = DepthProcessor()
    rclpy.spin(depth_processor)
    depth_processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()