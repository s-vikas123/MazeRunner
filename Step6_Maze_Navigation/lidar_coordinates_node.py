#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PointStamped, PoseStamped, Pose2D
from tf2_ros import Buffer, TransformListener, TransformException
import math
import transforms3d  # For quaternion to Euler angle conversion


class LidarMapCoordinatesNode(Node):
    def __init__(self):
        super().__init__('lidar_map_coordinates_node')

        # TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscribe to LIDAR scan topic
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10
        )

        self.get_logger().info("LIDAR map coordinate node initialized.")

    def lidar_callback(self, scan_msg):
        # Create a point in the LIDAR frame (base_link)
        point = PointStamped()
        point.header.frame_id = scan_msg.header.frame_id  # typically 'base_link' or similar
        point.header.stamp = scan_msg.header.stamp
        
        # Example coordinates and angle in base_link frame
        x_in_lidar = -0.85  # meters in front of the robot
        y_in_lidar = 0.0  # meters to the left of the robot
        theta_in_lidar = math.pi  # 45 degrees in radians
        
        point.point.x = x_in_lidar
        point.point.y = y_in_lidar
        point.point.z = 0.0

        try:
            # Look up transform from base_link frame to map frame
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link', #point.header.frame_id,  # typically 'base_link'
                rclpy.time.Time()
            )
            self.get_logger().info(
                f"frame:{point.header.frame_id}"
            )
            # Transform the point to map frame
            from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_point
            point_map = do_transform_point(point, transform)

            # Now transform the angle (theta) to map frame
            # Get the robot's orientation in map frame from the transform
            robot_orientation = transform.transform.rotation
            # Convert quaternion to Euler angles (yaw)
            _, _, robot_yaw = transforms3d.euler.quat2euler(
                [robot_orientation.w, robot_orientation.x, robot_orientation.y, robot_orientation.z])
            
            # The angle in map frame is the robot's yaw plus the angle in base_link frame
            theta_in_map = robot_yaw + theta_in_lidar
            
            self.get_logger().info(
                f"Transformed Position (map frame): x={point_map.point.x:.2f}, y={point_map.point.y:.2f}"
            )
            self.get_logger().info(
                f"Transformed Angle (map frame): {math.degrees(theta_in_map):.2f} degrees"
            )

            # Create a full pose in map frame (position and orientation)
            pose_map = PoseStamped()
            pose_map.header.frame_id = 'map'
            pose_map.header.stamp = scan_msg.header.stamp

            # Position
            pose_map.pose.position.x = point_map.point.x
            pose_map.pose.position.y = point_map.point.y
            pose_map.pose.position.z = 0.0  # Assuming 2D

            # Orientation (convert theta_in_map back to quaternion)
            quat = transforms3d.euler.euler2quat(0, 0, theta_in_map)
            pose_map.pose.orientation.x = quat[1]
            pose_map.pose.orientation.y = quat[2]
            pose_map.pose.orientation.z = quat[3]
            pose_map.pose.orientation.w = quat[0]
            
            self.get_logger().info(f"Full Pose in map frame:")
            self.get_logger().info(f"Position: x={pose_map.pose.position.x:.2f}, y={pose_map.pose.position.y:.2f}")
            self.get_logger().info(f"Orientation: yaw={math.degrees(theta_in_map):.2f}°")
            self.get_logger().info(f"Quaternion: x={pose_map.pose.orientation.x:.2f}, y={pose_map.pose.orientation.y:.2f}, z={pose_map.pose.orientation.z:.2f}, w={pose_map.pose.orientation.w:.2f}")

        except TransformException as e:
            self.get_logger().warn(f'Could not transform point: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = LidarMapCoordinatesNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
