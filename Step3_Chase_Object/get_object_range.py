#!/usr/bin/env python3
""" 
get_object_range.py

Authors: Sai Vikas Satyanarayana, Keerthi Panguluri
Date: 2/6/25

Description:
This ROS 2 node subscribes to object coordinates (/object/coordinates_lidar_frame) 
and LIDAR scan data (/scan), calculates the object's range based on its 
angular position, and publishes the result to the /object/range topic.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
from math import radians
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
import time
import numpy as np



class GetObjectRangeNode(Node):
    def __init__(self):
        super().__init__('get_object_range')
        self.get_logger().info("Starting Get Object Range Node...")
        
        # Subscribe to object coordinates (angular position in the LIDAR frame)
        self.coordinates_subscriber = self.create_subscription(
            Point,
            '/object/coordinates_lidar_frame',
            self.coordinates_callback,
            10
        )
        # Set up QoS Profiles for LIDAR
        lidar_qos_profile = QoSProfile(
            depth=5,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )
        # Subscribe to LIDAR data
        self.lidar_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            lidar_qos_profile
        )
        
        # Publisher to publish the object range and angular position
        self.range_publisher = self.create_publisher(
            Point,
            '/object/range',
            10
        )
        
        # Store received data
        self.lidar_data = None
        self.object_angle = None  # Only one angle needed

    def coordinates_callback(self, msg):
        """ Stores object angular position and processes range if LIDAR data is available. """
        self.object_angle = msg.x  # Only x is relevant (angle in radians)
        if self.lidar_data is not None:
            self.process_object_range()

    def lidar_callback(self, msg):
        """ Stores LIDAR data and processes object range if angle is available. """
        self.lidar_data = msg
        
        if self.object_angle is not None:
            self.process_object_range()

    def process_object_range(self):
        """ Matches object angle with LIDAR scan to get object range. """
        angle_min = self.lidar_data.angle_min
        angle_increment = self.lidar_data.angle_increment
        ranges = self.lidar_data.ranges

        # Compute LIDAR scan index for given object angle
        index = int((-(self.object_angle) - angle_min) / angle_increment)

        # Get values from index-2 to index+2 within valid bounds
        window_values = []
        for i in range(5):
            window_values.append(ranges[i+index-2])
        window_values = np.array(window_values)

        start, end = index - 2,  index + 2

        # Filter out NaNs
        valid_values = window_values[~np.isnan(window_values)]
        
        # Compute the average if valid values exist
        object_distance = float(np.mean(valid_values)) if valid_values.size > 0 else float('inf')
        
        # Publish object angular position and range
        object_range = Point(x=self.object_angle, y=0.0, z=object_distance)
        self.range_publisher.publish(object_range)

        self.get_logger().info(f"Object at angle {self.object_angle:.2f} rad, distance {object_distance:.2f} m")

def main(args=None):
    rclpy.init(args=args)
    node = GetObjectRangeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
