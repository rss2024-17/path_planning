#!/usr/bin/env python3
import numpy as np
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDrive
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker
from std_msgs.msg import Float32
from wall_follower.visualization_tools import VisualizationTools

##ADD TO SETUP.PY!! 

class Safety(Node):
    def __init__(self):
        super().__init__("safety_controller")
        # Declare parameters to make them available for use
        # self.declare_parameter("scan_topic", "default")
        # self.declare_parameter("drive_topic", "default")
        # self.declare_parameter("side", "default")
        # self.declare_parameter("velocity", "default")
        # self.declare_parameter("desired_distance", "default")

        # # Fetch constants from the ROS parameter server
        # self.SCAN_TOPIC = self.get_parameter('scan_topic').get_parameter_value().string_value
        # self.DRIVE_TOPIC = self.get_parameter('drive_topic').get_parameter_value().string_value
        # self.SIDE = self.get_parameter('side').get_parameter_value().integer_value
        # self.VELOCITY = self.get_parameter('velocity').get_parameter_value().double_value
        # self.DESIRED_DISTANCE = self.get_parameter('desired_distance').get_parameter_value().double_value

        # self.declare_parameter('safety_controller/scan_topic', '/scan')
        # self.SCAN_TOPIC = self.get_parameter('safety_controller/scan_topic').get_parameter_value().string_value

	
        # self.subscription = self.create_subscription(
        #     LaserScan,
        #     self.SCAN_TOPIC,
        #     self.listener_callback,
        #     10)
        # self.subscription  # prevent unused variable warning
        # self.publisher_drive = self.create_publisher(AckermannDriveStamped, self.DRIVE_TOPIC, 10)

        self.vehicle_speed = 0.0
        self.stopping_dist = 0.2

        self.subscription_scan = self.create_subscription(LaserScan, 'scan', self.scan_listener_callback, 10)
        self.subscription_scan  # prevent unused variable warning
        self.subscription_drive = self.create_subscription(AckermannDriveStamped, '/vesc/high_level/ackermann_cmd_mux/output', self.drive_listener_callback, 10)
        self.subscription_drive  # prevent unused variable warning
        self.publisher_drive = self.create_publisher(AckermannDriveStamped, '/vesc/low_level/ackermann_cmd_mux/input/safety', 10)

    def scan_listener_callback(self, laser_msg):
        # self.get_logger().info('I heard: "%s"' % laser_msg.ranges[len(laser_msg.ranges)//2])
        msg_drive = AckermannDriveStamped()
        msg_drive.header = laser_msg.header       
        msg_drive.drive = AckermannDrive()

        if (min(laser_msg.ranges) < self.stopping_dist):
            msg_drive.drive.speed = 0.0
            self.get_logger().info('STOPPING')
    
        self.publisher_drive.publish(msg_drive)
    

    def drive_listener_callback(self, drive_msg):
        self.vehicle_speed = drive_msg.drive.speed
        # for if we implement the safety controller with speed as a factor


def main():
    
    rclpy.init()
    safety_controller = Safety()
    rclpy.spin(safety_controller)
    safety_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    
        


        


    


        



