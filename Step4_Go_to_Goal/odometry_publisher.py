#!/usr/bin/env python3
""" 
odometry_publisher.py

Author: Sai Vikas Satyanarayana, Keerthi Panguluri
Date: 3/24/25

 """
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image
from std_msgs.msg import Float32            
from cv_bridge import CvBridge
import numpy as np
import cv2
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
import math     
from geometry_msgs.msg import Point                           

class LidarVisualizer(Node):
    def __init__(self):
        super().__init__('lidar_visualizer')
        # self.d = 0.5
        # Initialize attributes
        self.u_gtg = None
        self.u_ao = None
        self.dot_product = 0.0

        # Set up QoS Profiles for passing images over WiFi
        lidar_qos_profile = QoSProfile(
            depth=5,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )
        
        # Subscribe to LIDAR data
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            lidar_qos_profile
        ) 
        self.u = np.array([1.0, 0.0, 0.0])  # Default heading in x-direction
        self.u_gtg_sub = self.create_subscription(Point, '/u_gtg', self.u_gtg_callback, 10)
        self.u_ao_sub = self.create_subscription(Point, '/u_ao', self.u_ao_callback, 10)
        self.u_sub = self.create_subscription(Point, '/u', self.u_callback, 10)
        # self.u = np.array([1.0, 0.0, 0.0])  # Default heading in x-direction
        # self.u = None


        # Lidar range publisher (using LaserScan message type)
        self.lidar_pub = self.create_publisher(
            LaserScan, 
            '/lidar_range', 
            10
        )

        # Publisher for the obstacle distance (new publisher)
        self.obstacle_pub = self.create_publisher(
            Float32,
            '/obstacle_distance',
            10
        )

        self.obstacle_coord_pub = self.create_publisher(
            Point,
            '/obstacle_coords',
            10
        )

        self.bridge = CvBridge()

    def u_gtg_callback(self, msg):
        self.u_gtg = np.array([msg.x, msg.y, msg.z])

    def u_ao_callback(self, msg):
        self.u_ao = np.array([msg.x, msg.y, msg.z])

    def u_callback(self, msg):
        self.u = np.array([msg.x, msg.y, msg.z])


    def lidar_callback(self, msg):
        self.lidar_data = msg
        ranges = np.array(msg.ranges)

        # I am just using OpenCV for debugging

        # Generate LiDAR image
        img_size = 500
        img = np.ones((img_size, img_size, 3), dtype=np.uint8) * 255
        center = (img_size // 2, img_size // 2)
        max_range = msg.range_max

        angle_increment = msg.angle_increment
        angle_min = msg.angle_min
        angle_max = msg.angle_max
        # self.get_logger().info(f'min: {angle_min}, max: {angle_max}')

        # Replace NaN values with the average of their neighbors
        for i in range(1, len(ranges) - 1):
            if np.isnan(ranges[i]):  # Only check for NaN values
                left = ranges[i - 1] if not np.isnan(ranges[i - 1]) else None
                right = ranges[i + 1] if not np.isnan(ranges[i + 1]) else None

                if left is not None and right is not None:
                    ranges[i] = (left + right) / 2  # Average of left and right
                elif left is not None:
                    ranges[i] = left  # Use left value if right is NaN
                elif right is not None:
                    ranges[i] = right  # Use right value if left is NaN

        # Visualize LiDAR data on an image
        for i, r in enumerate(ranges):
            if np.isinf(r) or np.isnan(r) or r > max_range:
                continue

            angle = angle_min + i * angle_increment
            
            # Convert LiDAR polar coordinates to Cartesian coordinates for display
            x = int(center[0] - (r / max_range) * (img_size // 2) * np.sin(angle))
            y = int(center[1] - (r / max_range) * (img_size // 2) * np.cos(angle))
            

            cv2.circle(img, (x, y), 2, (0, 0, 255), -1)

        # --- Code for Obstacle Distance Calculation ---
        # Define the heading range +-pi/x

        # Compute angles for each measurement
        angles = angle_min + np.arange(len(ranges)) * angle_increment
        angles_dum = (angles + np.pi) % (2 * np.pi) - np.pi

        # Define heading ranges for the three cases
        # heading_range1 = math.pi / 15
        # heading_range2 = math.pi / 2
        # heading_range3 = math.pi / 2

        # # Case 1
        # valid_mask1 = (angles_dum >= -heading_range1) & (angles_dum < heading_range1) & np.isfinite(ranges) & (ranges <= max_range)
        # valid_ranges1 = ranges[valid_mask1]
        # valid_angles1 = angles[valid_mask1]

        # if valid_ranges1.size > 0:
        #     obstacle_distance1 = min(np.mean(valid_ranges1), np.median(valid_ranges1))
        # else:
        #     obstacle_distance1 = max_range

        # # Case 2
        # valid_mask2 = (angles_dum >= 0) & (angles_dum < heading_range2) & np.isfinite(ranges) & (ranges <= max_range)
        # valid_ranges2 = ranges[valid_mask2]
        # valid_angles2 = angles[valid_mask2]

        # if valid_ranges2.size > 0:
        #     obstacle_distance2 = min(np.mean(valid_ranges2), np.median(valid_ranges2))
        # else:
        #     obstacle_distance2 = max_range

        # # Case 3
        # valid_mask3 = (angles_dum >= -heading_range3) & (angles_dum < 0) & np.isfinite(ranges) & (ranges <= max_range)
        # valid_ranges3 = ranges[valid_mask3]
        # valid_angles3 = angles[valid_mask3]

        # if valid_ranges3.size > 0:
        #     obstacle_distance3 = min(np.mean(valid_ranges3), np.median(valid_ranges3))
        # else:
        #     obstacle_distance3 = max_range

        # # Find the final obstacle distance and its corresponding angle
        # obstacle_distances = [obstacle_distance1, obstacle_distance2, obstacle_distance3]
        # valid_angles_list = [valid_angles1, valid_angles2, valid_angles3]

        # Define heading ranges for the three cases
        heading_range1_deg = 12
        heading_range1 = math.pi / heading_range1_deg
        # Parameters
        num_iterations = (360 // (2 * heading_range1_deg))  # Total iterations based on step size
        obstacle_distances = np.full(num_iterations, max_range)  # Preallocate with max_range
        valid_angles_list = np.full(num_iterations, 0.0)  # Preallocate list for angles

        for i in range(num_iterations):
            start_angle = -heading_range1 + i * (2 * heading_range1)
            end_angle = start_angle + (2 * heading_range1)

            if end_angle > np.pi:
                start_angle -= 2 * np.pi
                end_angle -= 2 * np.pi

            valid_mask = (angles_dum >= start_angle) & (angles_dum < end_angle) & np.isfinite(ranges) & (ranges <= max_range)
            valid_ranges = ranges[valid_mask]
            valid_angles = angles[valid_mask]

            if valid_ranges.size > 0:
                obstacle_distances[i] = min(np.mean(valid_ranges), np.median(valid_ranges))
                valid_angles_list[i] = valid_angles[np.argmin(valid_ranges)]  # Correct indexing
            else:
                valid_angles_list[i] = 0.0  # No valid angle found

        min_index = np.argmin(obstacle_distances)
        obstacle_distance = obstacle_distances[min_index]
        obstacle_angle = valid_angles_list[min_index]
        self.get_logger().info(f'Angle: {obstacle_angle*180/math.pi}')

        # if self.u_gtg is not None and self.u_ao is not None:
        #     self.dot_product = np.dot(self.u_gtg, self.u_ao)
        #     # self.get_logger().info(f'Dot Product: {dot_product}')

        # if self.dot_product > 0.8:
        #     obstacle_distance = obstacle_distances[0]
        #     obstacle_angle = valid_angles_list[0]
        # else:
        #     min_index = np.argmin(obstacle_distances)
        #     obstacle_distance = obstacle_distances[min_index]
        #     obstacle_angle = valid_angles_list[min_index]

        min_index = np.argmin(obstacle_distances)
        obstacle_distance = obstacle_distances[min_index]
        obstacle_angle = valid_angles_list[min_index]


        obstacle_x = float(obstacle_distance*np.cos(obstacle_angle))
        obstacle_y = float(obstacle_distance*np.sin(obstacle_angle))
        

        obstacle_point = Point()
        obstacle_point.x = obstacle_x
        obstacle_point.y = obstacle_y
        obstacle_point.z = 0.0
        self.obstacle_coord_pub.publish(obstacle_point)

        self.get_logger().info(f'x: {obstacle_point.x}, y: {obstacle_point.y}')

        obstacle_x_fig = int(center[0] - (obstacle_distance / max_range) * (img_size // 2) * np.sin(obstacle_angle))
        obstacle_y_fig = int(center[0] - (obstacle_distance / max_range) * (img_size // 2) * np.cos(obstacle_angle))


        # Draw a green circle to indicate the detected obstacle point
        cv2.circle(img, (obstacle_x_fig, obstacle_y_fig), 6, (0, 255, 0), -1)
        cv2.putText(img, "Obstacle", (obstacle_x_fig + 8, obstacle_y_fig), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        # Draw a small blue circle at the robot's position
        cv2.circle(img, center, 5, (255, 0, 0), -1)

        # Scale factor for visualization
        scale_factor = (img_size // 2) * 0.5  # 80% of half image size

        self.get_logger().info(f'ux = {self.u[0]}')

        # Compute end point of the heading vector
        end_x = int(center[0] - self.u[1] * scale_factor)
        end_y = int(center[1] - self.u[0] * scale_factor)

        # Draw the heading direction
        cv2.arrowedLine(img, center, (end_x, end_y), (255, 0, 0), 2, tipLength=0.2)


        # Display the image using OpenCV
        cv2.imshow("LiDAR Visualization", img)
        if cv2.waitKey(1) & 0xFF == 27:  # Press 'Esc' to exit
            self.destroy_node()
            cv2.destroyAllWindows()

        # Publish the corrected lidar data
        msg.ranges = ranges.tolist()
        self.lidar_pub.publish(msg)

        # Publish the obstacle distance
        obstacle_msg = Float32()
        obstacle_msg.data = float(obstacle_distance)
        self.obstacle_pub.publish(obstacle_msg)
        self.get_logger().info(f'distance = {obstacle_distance}')

        

    def destroy_node(self):
        super().destroy_node()
        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = LidarVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()