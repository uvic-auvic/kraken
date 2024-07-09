#!/usr/bin/env python

"""
Peripheral controller node.
"""

import sys

sys.path.append("/home/auvic/kraken/src/kraken/kraken/include")

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class PeripheralController(Node):

    def __init__(self):
    
        super().__init__('peripheral_controller')
        
        self.logger = self.get_logger()
        self.publisher = self.create_publisher(Float32MultiArray, 'peripheral_control', 10)

		# Subscribers
		self.depth_sub = self.create_subscription(String, 'depth', self.depth_callback, 10)
		self.imu_sub = self.create_subscription(String, 'imu', self.imu_callback, 10)
		self.control_sub = self.create_subscription(String, 'control_system', self.control_callback, 10)


        # Prevent unused variable warning
        self.depth_sub
		self.imu_sub
		self.control_sub

    def depth_callback(self, msg):
        pass

    def imu_callback(self, msg):
        pass

    def control_callback(self, msg):
        pass

def main(args=None):
    rclpy.init(args=args)

    publisher = PeripheralController()

    rclpy.spin(publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()