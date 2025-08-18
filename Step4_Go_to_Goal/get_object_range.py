#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from geometry_msgs.msg import Point
import numpy as np
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
import cv2
from sklearn.cluster import DBSCAN
from scipy.spatial import ConvexHull

class LidarSegmenter(Node):
    def __init__(self):
        super().__init__('lidar_segmenter')

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
        self.window_name = "LiDAR Visualization"
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)

        # Initialize Kalman filter
        self.kf = cv2.KalmanFilter(4, 2)  # 4 states (x, y, vx, vy), 2 measurements (x, y)
        self.kf.measurementMatrix = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], np.float32)
        self.kf.transitionMatrix = np.array([[1, 0, 1, 0], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]], np.float32)
        self.kf.processNoiseCov = np.eye(4, dtype=np.float32) * 1e-4  # Small process noise


    def lidar_callback(self, msg):
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))
        max_range = msg.range_max

        # Filter valid LiDAR points
        valid_mask = np.isfinite(ranges) & (ranges <= max_range)
        valid_ranges = ranges[valid_mask]
        valid_angles = angles[valid_mask]

        if valid_ranges.size == 0:
            return

        # Convert to Cartesian coordinates
        x_coords = valid_ranges * np.cos(valid_angles)
        y_coords = valid_ranges * np.sin(valid_angles)
        points = np.column_stack((x_coords, y_coords))

        # Apply DBSCAN for clustering
        clustering = DBSCAN(eps=0.2, min_samples=5).fit(points)
        labels = clustering.labels_

        # Identify the nearest obstacle cluster
        unique_labels = set(labels)
        min_distance = max_range
        nearest_point = None
        
        for label in unique_labels:
            if label == -1:
                continue  # Ignore noise points
            cluster_points = points[labels == label]
            cluster_distances = np.linalg.norm(cluster_points, axis=1)
            min_cluster_dist = np.min(cluster_distances)
            
            if min_cluster_dist < min_distance:
                min_distance = min_cluster_dist
                nearest_point = cluster_points[np.argmin(cluster_distances)]

        if nearest_point is not None:
            measurement = np.array([[nearest_point[0]], [nearest_point[1]]], np.float32)
            self.kf.correct(measurement)
            prediction = self.kf.predict()
            
            obstacle_point = Point()
            obstacle_point.x = float(prediction[0])
            obstacle_point.y = float(prediction[1])
            obstacle_point.z = 0.0
            self.obstacle_coord_pub.publish(obstacle_point)

            self.get_logger().info(f'Smoothed Obstacle at (x: {obstacle_point.x}, y: {obstacle_point.y})')

        # Visualization
        self.display_lidar(points, labels)
        
        # Calculate and publish edge coordinates for each cluster
        for label in unique_labels:
            if label == -1:
                continue  # Ignore noise points
            cluster_points = points[labels == label]

            # Compute the convex hull of the cluster points
            if len(cluster_points) > 2:  # Convex hull requires at least 3 points
                hull = ConvexHull(cluster_points)
                hull_points = cluster_points[hull.vertices]

                # Publish edge coordinates (vertices of the convex hull)
                for point in hull_points:
                    edge_point = Point()
                    edge_point.x = float(point[0])
                    edge_point.y = float(point[1])
                    edge_point.z = 0.0
                    self.obstacle_coord_pub.publish(edge_point)
                    self.get_logger().info(f'Edge point: (x: {edge_point.x}, y: {edge_point.y})')

        # Get the number of clusters (excluding noise)
        num_clusters = len(set(labels)) - (1 if -1 in labels else 0)
        self.get_logger().info(f'Number of clusters detected: {num_clusters}')


    def display_lidar(self, points, labels):
        img_size = 500  # Image resolution
        scale = 50      # Scaling factor for visualization
        center = img_size // 2

        # Create a black background
        img = np.zeros((img_size, img_size, 3), dtype=np.uint8)

        # Assign unique colors to each cluster
        unique_labels = set(labels)
        colors = {label: (np.random.randint(100, 255), np.random.randint(100, 255), np.random.randint(100, 255)) 
                  for label in unique_labels if label != -1}

        for (point, label) in zip(points, labels):
            px = int(center + point[0] * scale)
            py = int(center - point[1] * scale)  # Flip y for visualization

            if label == -1:
                color = (255, 255, 255)  # Noise points in white
            else:
                color = colors[label]

            cv2.circle(img, (px, py), 2, color, -1)

        # Draw center point
        cv2.circle(img, (center, center), 5, (0, 0, 255), -1)  # Robot position in red

        # Display the image
        cv2.imshow(self.window_name, img)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = LidarSegmenter()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == '__main__':
    main()