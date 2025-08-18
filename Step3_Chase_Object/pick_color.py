#!/usr/bin/env python3
""" 
pick_color.py

Author: Sai Vikas Satyanarayana, Keerthi Panguluri
Date: 2/6/25

Code Description:
This ROS 2 node subscribes to a compressed video stream (`/image_raw/compressed`), 
processes the received images, and allows users to pick an HSV color from the image 
by clicking on it. The selected color is then published to the `/picked_hsv` topic.

 """
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Point
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy

import numpy as np
import cv2
from cv_bridge import CvBridge

class MinimalVideoSubscriber(Node):

    def __init__(self):        
        super().__init__('minimal_video_subscriber')

        # Set Parameters
        self.declare_parameter('show_image_bool', True)
        self.declare_parameter('window_name', "Raw Image")

        self._display_image = bool(self.get_parameter('show_image_bool').value)
        self._titleOriginal = self.get_parameter('window_name').value 

        if self._display_image:
            cv2.namedWindow(self._titleOriginal, cv2.WINDOW_AUTOSIZE)
            cv2.moveWindow(self._titleOriginal, 50, 50) 
            cv2.setMouseCallback(self._titleOriginal, self.pick_color)

        # Set up QoS Profiles for passing images over WiFi
        image_qos_profile = QoSProfile(
            depth=5,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )

        self._video_subscriber = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self._image_callback,
            image_qos_profile
        )

        self.hsv_publisher = self.create_publisher(Point, '/picked_hsv', 10)

        self.bridge = CvBridge()  # Create a CvBridge instance
        self.hsv = None  
        self.lower_bound = np.array([0, 0, 0])
        self.upper_bound = np.array([179, 255, 255])

    def _image_callback(self, msg):    
        self._imgBGR = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        self.hsv = cv2.cvtColor(self._imgBGR, cv2.COLOR_BGR2HSV)

        if self._display_image:
            self.show_image(self._imgBGR)

    def pick_color(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and self.hsv is not None:
            kernel_size = 25
            half_k = kernel_size // 2
            y_min, y_max = max(0, y - half_k), min(self.hsv.shape[0], y + half_k + 1)
            x_min, x_max = max(0, x - half_k), min(self.hsv.shape[1], x + half_k + 1)

            neighborhood = self.hsv[y_min:y_max, x_min:x_max]
            neighborhood = cv2.GaussianBlur(neighborhood, (5, 5), 0)
            median_hsv = np.median(neighborhood.reshape(-1, 3), axis=0).astype(np.uint8)

            self.get_logger().info(f"Picked HSV Color: {median_hsv}")

            hsv_msg = Point()
            hsv_msg.x, hsv_msg.y, hsv_msg.z = map(float, median_hsv)
            self.hsv_publisher.publish(hsv_msg)

    def show_image(self, img):
        cv2.imshow(self._titleOriginal, img)
        key = cv2.waitKey(50)  
        if key == ord('q'):
            cv2.destroyAllWindows()
            rclpy.shutdown()

def main():
    rclpy.init()
    video_subscriber = MinimalVideoSubscriber()

    try:
        rclpy.spin(video_subscriber)        
    except KeyboardInterrupt:
        pass
    finally:
        video_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()