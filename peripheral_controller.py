#!/usr/bin/env python

"""
Peripheral controller node.
"""
import sys
import math

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

        self.depth_sub  # prevent unused variable warning
		self.imu_sub
		self.control_sub
        
        timer_period = 0.5 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

		# Current position
		self.position = [0, 0, 1] # x (forward), y (right), z (up)

		# Target position
		self.target = [0, 0, 1]

		# Difference between target and current position
		self.delta_position = [0, 0, 0]

	# Receive depth data
	def depth_callback(self, msg):
		self.logger().info(msg.data)

		depth = msg.data[0]
		pressure = msg.data[1]
		temperature = msg.data[2]

		self.position[2] = depth

	# Receive imu data
	def imu_callback(self, msg):
		self.logger().info(msg.data)

		position = msg.position # x (forward), y (right)

		self.position[0] = position[0]
		self.position[1] = position[1]
	
	# Receive control
	def control_callback(self, msg):
		self.logger().info(msg.data)

		action = msg.action
		direction = msg.direction
		position = msg.position

		# Set target
		if action == "move":
			if self.target == None:
				self.target = self.position
			if direction == "forward":
				self.target[0] += position

			elif direction == "backward":
				self.target[0] -= position

			elif direction == "right":
				self.target[1] += position

			elif direction == "left":
				self.target[1] -= position

			elif direction == "up":
				self.target[2] += position

			elif direction == "down":
				self.target[2] -= position

    def timer_callback(self):
        
		self.delta_position[0] = self.target[0] - self.position[0]
		self.delta_position[1] = self.target[1] - self.position[1]
		self.delta_position[2] = self.target[2] - self.position[2]

		# Determine motor movement

		# If the position is within a reasonable distance to the target, done
		if self.delta_position[0] ** 2 + self.delta_position[1] ** 2 + self.delta_position[2] ** 2 < 0.01:
			self.target = None			
			# Send done signal


def main(args=None):
    rclpy.init(args=args)

    peripheral_controller = PeripheralController()

    rclpy.spin(peripheral_controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    peripheral_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
