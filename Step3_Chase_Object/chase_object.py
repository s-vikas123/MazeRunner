#!/usr/bin/env python3
""" 
chase_object.py

Author: Sai Vikas Satyanarayana, Keerthi Panguluri
Date: 2/6/25

Code Description:
This ROS 2 node subscribes to object coordinates (/object/coordinates) 
and LIDAR range data (/object/range). It computes the necessary velocity 
commands to move a robot towards the object, rotating to center it and 
adjusting the robot’s speed to maintain a desired distance. The 
calculated velocity commands are published to the cmd_vel topic for the 
robot's movement.

 """
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point


class ChaseObject(Node):
    def __init__(self):
        super().__init__('rotate_robot')
        
        # Declare parameters
        self.declare_parameter('image_width', 320)  # Default width of camera frame
        self.image_width = self.get_parameter('image_width').value
        
        # Define subscribers for object coordinates and lidar distance
        self.subscription = self.create_subscription(
            Point,
            '/object/coordinates',  # Topic from find_object node
            self.object_callback,
            10
        )      
        self.linear_subscription = self.create_subscription(
            Point, '/object/range', self.lidar_callback, 10
        )

        # Define a publisher for velocity commands
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 5)
        
        # Object tracking variables
        self.object_x = None
        self.object_distance = None 

        self.get_logger().info("rotate_robot node started!")

        # Timer for steady movement updates
        self.timer = self.create_timer(0.1, self.movement_callback) 

    def object_callback(self, msg):
        self.get_logger().info(f"Received object coordinates: x={msg.x}, y={msg.y}, z={msg.z}")
        self.object_x = msg.x  # Store X-coordinate of the object

    def lidar_callback(self, msg):
        self.get_logger().info(f"Received lidar coordinates: z={msg.z}")
        self.object_distance = msg.z  # Store object distance

    def movement_callback(self):
        if self.object_x is None or self.object_distance is None:
            return  # Wait until both values are available

        # Calculate center of the image
        center_x = self.image_width / 2  
        angular_tolerance = 15  # Pixels tolerance to avoid small oscillations

        twist = Twist()
        
        # Angular control (rotation to center the object)
        Kp_angular = 0.01 
        error_angular = center_x - self.object_x  
        twist.angular.z = max(min(Kp_angular * error_angular, 1.5), -1.5)  # Cap max rotation speed

        if abs(error_angular) < angular_tolerance:
            twist.angular.z = 0.0
            self.get_logger().info("Object centered, stopping rotation")
        
        # Linear control (move towards the object)
        Kp_linear = 0.5
        desired_distance = 0.35  # Desired stopping distance from the object
        distance_tolerance = 0.01  # Tolerance for stopping movement
        error_linear = self.object_distance - desired_distance
        
        
        twist.linear.x = max(min(Kp_linear * error_linear, 0.2), -0.2)  # Cap max speed
        
        self.publisher.publish(twist)
        self.get_logger().info(f"Received lidar coordinates: forward vel={twist.linear.x}, angular vel = {twist.angular.z}")


def main(args=None):
    rclpy.init(args=args)
    node = ChaseObject()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
