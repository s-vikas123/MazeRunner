#!/usr/bin/env python3
""" 
detect_object.py

Author: Sai Vikas Satyanarayana, Keerthi Panguluri
Date: 2/6/25

Code Description:
This ROS 2 node subscribes to a compressed image topic and a picked HSV 
color topic, processes the image to detect object of the selected color, 
converts the pixel coordinates into the angle position on the lidar frame 
and publishes the detected object's coordinates in the pixel value and 
the calculated angle position on the lidar frame.

 """
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import numpy as np

class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('detect_object')
        self.get_logger().info("Starting Object Detection Node...")
        self.bridge = CvBridge()

        # Subscribe to Raspberry Pi Camera Node
        self.image_subscriber = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.image_callback,
            10
        )

        self.color_subscriber = self.create_subscription(
            Point,  
            '/picked_hsv',  
            self.color_callback,
            10
        )
        
        # Publish object coordinates in LIDAR-compatible units
        self.lidar_coordinates_publisher = self.create_publisher(
            Point,
            '/object/coordinates_lidar_frame',
            10
        )

        self.coordinates_publisher = self.create_publisher(
            Point,
            '/object/coordinates',
            10
        )

        # HSV Color Bounds
        self.lower_bound = np.array([0, 0, 0], dtype=np.uint8)
        self.upper_bound = np.array([179, 255, 255], dtype=np.uint8)
        self.picked_color_image = np.zeros((200, 200, 3), dtype=np.uint8)
        self.hsv = None

        # Camera Parameters (Adjust as needed)
        self.camera_fov = 60  # Field of view in degrees
        self.image_width = 320  # Camera resolution width in pixels

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

                K = np.array([
                    [265.23347032884044, 0, 160.5],
                    [0, 265.23347032884044, 120.5],
                    [0, 0, 1]
                ]) # get this array from ros2 topic echo /simulated_camera/camera_info

                FOV_x = np.deg2rad(62.2)  # FOV in degrees
                image_width = 320       # image width (resolution)

                # Convert pixel X-coordinate to angle (radians)
                angle_radians_x = ((center_x-160.0) * FOV_x) / (image_width)
                angle_radians_y = 0.0

                # coordinates in LIDAR reference frame
                lidar_object_coordinates = Point(x=angle_radians_x, y=angle_radians_y, z=0.0)
                
        # Publish coordinates
        try:
            if object_coordinates:
                self.lidar_coordinates_publisher.publish(lidar_object_coordinates)
                self.coordinates_publisher.publish(object_coordinates)
        except Exception as e:
            self.get_logger().error(f"Error publishing messages: {e}")

        

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
