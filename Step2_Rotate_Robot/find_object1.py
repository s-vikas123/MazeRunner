#!/usr/bin/env python3
""" 
find_object1.py

Author: Sai Vikas Satyanarayana, Keerthi Panguluri
Date: 2/6/25

Code Description:
This ROS 2 node subscribes to a compressed image topic and a picked HSV 
color topic, processes the image to detect objects of the selected color, 
and publishes the processed image along with the detected object's 
coordinates.
 """

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage 
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import numpy as np

class ColorDetectionNode(Node):
    def __init__(self):
        super().__init__('color_detection_node')
        self.get_logger().info("Starting Color Detection Node...")
        self.bridge = CvBridge()

        # Create subscriptions and publishers
        self.image_subscriber = self.create_subscription(
            CompressedImage,  # Use CompressedImage instead of Image
            '/image_raw/compressed',  
            self.image_callback,
            10
        )

        # Subscribe to the HSV color picked topic
        self.color_subscriber = self.create_subscription(
            Point,  
            '/picked_hsv',  
            self.color_callback,
            10
        )

        self.image_publisher = self.create_publisher(
            CompressedImage,  # Use CompressedImage for the published topic
            '/processed/image/compressed',
            10
        )
        #publishing the coordinates for the 
        self.coordinates_publisher = self.create_publisher(
            Point,
            '/object/coordinates',
            10
        )

        # Initialize variables
        self.lower_bound = np.array([0, 0, 0], dtype=np.uint8)
        self.upper_bound = np.array([179, 255, 255], dtype=np.uint8)
        self.picked_color_image = np.zeros((200, 200, 3), dtype=np.uint8)
        self.hsv = None

    def color_callback(self, msg):    
        """Update color range based on received HSV values."""
        try:
            h, s, v = int(msg.x), int(msg.y), int(msg.z)

            # Define HSV bounds with a margin
            hue_margin = 10
            sat_margin = 50
            val_margin = 50

            self.lower_bound = np.array([max(0, h - hue_margin), max(0, s - sat_margin), max(0, v - val_margin)], dtype=np.uint8)
            self.upper_bound = np.array([min(179, h + hue_margin), min(255, s + sat_margin), min(255, v + val_margin)], dtype=np.uint8)

            self.get_logger().info(f"Updated HSV Range: Lower {self.lower_bound} - Upper {self.upper_bound}")

        except Exception as e:
            self.get_logger().error(f"Error in color callback: {e}")

    def image_callback(self, msg):
        try:
            frame = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')  # Use compressed_imgmsg_to_cv2
        except Exception as e:
            self.get_logger().error(f"Error converting ROS CompressedImage to OpenCV: {e}")
            return

        frame = np.uint8(np.clip(frame * 1.3, 0, 255))  # Equivalent to cv2.convertScaleAbs
        self.hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(self.hsv, self.lower_bound, self.upper_bound)
        mask = cv2.GaussianBlur(mask, (5, 5), 0)

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.dilate(mask, kernel, iterations=1)

        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        output_frame = frame.copy()
        object_coordinates = None
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest_contour) > 800:
                x, y, w, h = cv2.boundingRect(largest_contour)
                center_x, center_y = x + w // 2, y + h // 2
                cv2.rectangle(output_frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.circle(output_frame, (center_x, center_y), 5, (255, 0, 0), -1)
                cv2.putText(output_frame, f"({center_x}, {center_y})", (x, y - 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
                self.get_logger().info(f"Object Location: ({center_x}, {center_y})")
                object_coordinates = Point(x=float(center_x), y=float(center_y), z=0.0)

        # Publish processed image and coordinates
        try:
            processed_image_msg = self.bridge.cv2_to_compressed_imgmsg(output_frame)  # Use cv2_to_compressed_imgmsg
            self.image_publisher.publish(processed_image_msg)
            if object_coordinates:
                self.coordinates_publisher.publish(object_coordinates)
        except Exception as e:
            self.get_logger().error(f"Error publishing messages: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ColorDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
