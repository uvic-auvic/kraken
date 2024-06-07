#!/usr/bin/env python

"""
Depth sensor publisher node.
"""
import sys

sys.path.append("/home/auvic/kraken/src/kraken/kraken/include")

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

import imu

class IMU(Node):

    def __init__(self):
    
        super().__init__('imu')
        
        self.logger = self.get_logger()
        self.publisher = self.create_publisher(Float32MultiArray, 'imu', 10)
        
        timer_period = 0.5 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
     
    def poll_sensor(self):
        return imu.get_acceleration()
 
    def timer_callback(self):
        
        # Get data from the imu
        x, y, z = self.poll_sensor()

        if x or y or z:
            self.logger.info("x: %f, y: %f, z: %f" % (x, y, z))

            # Publish
            array = Float32MultiArray()
            array.data = [x, y, z]
            self.publisher.publish(array)


def main(args=None):
    rclpy.init(args=args)

    publisher = IMU()

    rclpy.spin(publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

