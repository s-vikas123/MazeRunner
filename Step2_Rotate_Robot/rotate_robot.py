#!/usr/bin/env python3
""" 
rotate_robot.py

Author: Sai Vikas Satyanarayana, Keerthi Panguluri
Date: 2/6/25

Code Description:
This ROS 2 node controls a robot's rotation based on the detected object's 
position in an image frame. It subscribes to an object coordinate topic 
(`/object/coordinates`) and publishes velocity commands to align the object 
to the center of the frame.

 """

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point

class RotateRobot(Node):
    def __init__(self):
        super().__init__('rotate_robot')
        
        # Declare parameters
        self.declare_parameter('image_width', 640)  # Default width of camera frame
        self.image_width = self.get_parameter('image_width').value
        
        # Define a subscriber for object coordinates
        self.subscription = self.create_subscription(
            Point,
            '/object/coordinates',  # Topic from find_object node
            self.object_callback,
            10
        )

        # Define a publisher for velocity commands
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 5)

        self.get_logger().info("rotate_robot node started!")

    def object_callback(self, msg):
        self.get_logger().info(f"Received object coordinates: x={msg.x}, y={msg.y}, z={msg.z}")
        object_x = msg.x  # X-coordinate of the object in the image

        # Calculate center of the image
        center_x = self.image_width / 4  
        tolerance = 15  # Pixels tolerance to avoid small oscillations

        twist = Twist()

        # Proportional control for smooth correction
        Kp = 0.002  
        error = center_x - object_x  
        twist.angular.z = Kp * error  

        if abs(error) < tolerance:
            twist.angular.z = 0.0
            self.get_logger().info("Object centered, stopping rotation")
            
            # Send multiple stop messages to ensure stop command is received
            for _ in range(3):
                self.publisher.publish(twist)
        
        else:
            direction = "left" if error > 0 else "right"
            self.get_logger().info(f"Turning {direction} with speed {twist.angular.z}")

        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = RotateRobot()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
