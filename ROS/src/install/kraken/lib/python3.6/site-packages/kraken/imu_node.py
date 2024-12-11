#!/usr/bin/env python

"""
IMU sensor publisher node.
"""
import sys
import threading
import argparse as ap

sys.path.append("/home/auvic/kraken/ROS/src/kraken/kraken/include")

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

import imu

imu_state = [[float(0), float(0), float(0)], [float(0), float(0), float(0)]]

class IMU(Node):

    def __init__(self):
    
        super().__init__('imu')
        parser = ap.ArgumentParser()
        parser.add_argument("-l", "--log", metavar="FILE", help="log data to a file", type=str)
        parser.add_argument("-v", "--verbose", help="display collected data", action="store_true")
        self.args = parser.parse_args()
        if self.args.log:
            self.log_file = open(self.args.log, "w")
            self.log_file.write("x,y,z,roll,pitch,yaw\n")
        
        self.logger = self.get_logger()
        self.publisher = self.create_publisher(Float32MultiArray, 'imu', 10)
        
        timer_period = 0.5 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
 
    def timer_callback(self):
        
        # Get data from the imu
        pos = imu_state[0]
        rot = imu_state[1]

        if self.args.verbose:
            self.logger.info("x: %f, y: %f, z: %f, Roll: %f, Pitch: %f, Yaw: %f" % (pos[0], pos[1], pos[2], rot[0], rot[1], rot[2]))
            
        if self.args.log:
            self.log_file.write(f"{pos[0]},{pos[1]},{pos[2]},{rot[0]},{rot[1]},{rot[2]}\n")

        # Publish
        array = Float32MultiArray()
        array.data = [pos[0], pos[1], pos[2], rot[0], rot[1], rot[2]]
        self.publisher.publish(array)


def main(args=None):
    rclpy.init(args=args)

    publisher = IMU()
    
    print("starting thread")
    t = threading.Thread(target=imu.position_thread, args=(imu_state,))
    t.start()

    rclpy.spin(publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

