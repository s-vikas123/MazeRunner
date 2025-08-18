#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image
from std_msgs.msg import Float32, String, Bool           
from cv_bridge import CvBridge
import numpy as np
import cv2
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
import math     
from geometry_msgs.msg import Point, Pose2D, Twist
import time 

class LidarVisualizer(Node):
    def __init__(self):
        super().__init__('lidar_visualizer')
       
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

        # Subscribe to detected class (direction)
        self.direction_sub = self.create_subscription(
            String,
            'detected_class',
            self.direction_callback,
            10
        )

        # Publisher for nav goal (x, y, theta) wrt to the lidar frame
        self.nav_goal_pub = self.create_publisher(
            Pose2D,
            '/lidar_goal',
            10
        )

        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        self.create_subscription(Pose2D, '/transferred_odom', self.pose_callback, 10)
        # This flag will be set to True when u came closer to the wall to read the direction
        self.came_closer = False
        self.last_class = None

        # Default heading (used for drawing arrows)
        self.u = np.array([1.0, 0.0, 0.0])  # Default heading in x-direction

        # Continuously updated value from the direction callback.
        self.current_class = 0
        self.theta_facing = 0.0
        self.vro_max = 0.8#1.0

        self.vlin_max = 0.1#0.2
        self.calib_vro_max = 1.2



        self.first = True
        self.first_x = 0.0
        self.first_y = 0.0
        self.first_theta = 0.0
        self.Kp_control_turn = 1.5


        self.current_pose = Pose2D()

        self.bridge = CvBridge()

    def pose_callback(self, msg):
        self.current_pose = msg

    def direction_callback(self, msg):
        try:
            self.current_class = int(msg.data)
            # Optionally, log the update:
            # self.get_logger().info(f"Received current_class: {self.current_class}")
        except ValueError:
            self.get_logger().warn(f"Could not convert '{msg.data}' to an integer.")


    def lidar_callback(self, msg):
        self.lidar_data = msg
        ranges = np.array(msg.ranges)

        # Generate LiDAR image for debugging using OpenCV.
        img_size = 500
        img = np.ones((img_size, img_size, 3), dtype=np.uint8) * 255
        center = (img_size // 2, img_size // 2)
        max_range = msg.range_max

        angle_increment = msg.angle_increment
        angle_min = msg.angle_min
        angle_max = msg.angle_max

        # Replace NaN values with the average of neighbors.
        for i in range(1, len(ranges) - 1):
            if np.isnan(ranges[i]):
                left = ranges[i - 1] if not np.isnan(ranges[i - 1]) else None
                right = ranges[i + 1] if not np.isnan(ranges[i + 1]) else None

                if left is not None and right is not None:
                    ranges[i] = (left + right) / 2
                elif left is not None:
                    ranges[i] = left
                elif right is not None:
                    ranges[i] = right

        # Visualize LiDAR data on an image.
        for i, r in enumerate(ranges):
            if np.isinf(r) or np.isnan(r) or r > max_range:
                continue

            angle = angle_min + i * angle_increment            
            # Convert polar to Cartesian coordinates for display.
            x = int(center[0] - (r / max_range) * (img_size // 2) * np.sin(angle))
            y = int(center[1] - (r / max_range) * (img_size // 2) * np.cos(angle))
            cv2.circle(img, (x, y), 2, (0, 0, 255), -1)

        # --- Obstacle Distance Calculation ---
        angles = angle_min + np.arange(len(ranges)) * angle_increment
        angles_dum = (angles + np.pi) % (2 * np.pi) - np.pi

        obstacle_range_deg = 12
        obstacle_range = math.radians(obstacle_range_deg)
        num_iterations = (360 // (2 * obstacle_range_deg))
        obstacle_distances = np.full(num_iterations, max_range)
        valid_angles_list = np.full(num_iterations, 0.0)

        for i in range(num_iterations):
            start_angle = -obstacle_range + i * (2 * obstacle_range)
            end_angle = start_angle + (2 * obstacle_range)

            if end_angle > np.pi:
                start_angle -= 2 * np.pi
                end_angle -= 2 * np.pi

            valid_mask = (angles_dum >= start_angle) & (angles_dum < end_angle) & np.isfinite(ranges) & (ranges <= max_range)
            valid_ranges = ranges[valid_mask]
            valid_angles = angles[valid_mask]

            if valid_ranges.size > 0:
                obstacle_distances[i] = min(np.mean(valid_ranges), np.median(valid_ranges))
                valid_angles_list[i] = valid_angles[np.argmin(valid_ranges)]
            else:
                valid_angles_list[i] = 0.0  

        min_index = np.argmin(obstacle_distances)
        obstacle_distance = obstacle_distances[min_index]
        obstacle_angle = valid_angles_list[min_index]

        obstacle_x = float(obstacle_distance * np.cos(obstacle_angle))
        obstacle_y = float(obstacle_distance * np.sin(obstacle_angle))
        
        obstacle_point = Point()
        obstacle_point.x = obstacle_x
        obstacle_point.y = obstacle_y
        obstacle_point.z = 0.0
        
        # self.get_logger().info(f'x: {obstacle_point.x}, y: {obstacle_point.y}')

        obstacle_x_fig = int(center[0] - (obstacle_distance / max_range) * (img_size // 2) * np.sin(obstacle_angle))
        obstacle_y_fig = int(center[1] - (obstacle_distance / max_range) * (img_size // 2) * np.cos(obstacle_angle))

        cv2.circle(img, (obstacle_x_fig, obstacle_y_fig), 6, (0, 255, 0), -1)
        cv2.putText(img, "Obstacle", (obstacle_x_fig + 8, obstacle_y_fig), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        cv2.circle(img, center, 5, (255, 0, 0), -1)
        scale_factor = (img_size // 2) * 0.5  # 80% of half image size

        # --- Draw the Heading Direction ---
        end_x = int(center[0] - self.u[1] * scale_factor)
        end_y = int(center[1] - self.u[0] * scale_factor)
        cv2.arrowedLine(img, center, (end_x, end_y), (255, 0, 0), 2, tipLength=0.2)

        # --- Compute and Draw the Normal Direction ---
        normal_vector = np.array([-self.u[1], self.u[0]])
        normal_end_x = int(center[0] - normal_vector[1] * scale_factor)
        normal_end_y = int(center[1] - normal_vector[0] * scale_factor)
        cv2.arrowedLine(img, center, (normal_end_x, normal_end_y), (0, 0, 255), 2, tipLength=0.2)
        cv2.putText(img, "Normal", (normal_end_x + 8, normal_end_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

        # ==== Compute min distance in heading direction ====
        heading_range = math.radians(10)
        heading_angle = math.atan2(self.u[1], self.u[0])
        angle_diff = (angles - heading_angle + np.pi) % (2 * np.pi) - np.pi
        heading_mask = (angle_diff >= -heading_range) & (angle_diff <= heading_range) & np.isfinite(ranges) & (ranges <= max_range)
        
        heading_min_distance = None
        if np.any(heading_mask):
            heading_ranges = ranges[heading_mask]
            heading_angles = angles[heading_mask]
            heading_min_idx = np.argmin(heading_ranges)
            heading_min_distance = heading_ranges[heading_min_idx]
            heading_min_angle = heading_angles[heading_min_idx]

            heading_x_fig = int(center[0] - (heading_min_distance / max_range) * (img_size // 2) * np.sin(heading_min_angle))
            heading_y_fig = int(center[1] - (heading_min_distance / max_range) * (img_size // 2) * np.cos(heading_min_angle))

            cv2.circle(img, (heading_x_fig, heading_y_fig), 6, (0, 255, 255), -1)
            cv2.putText(img, "Heading Min", (heading_x_fig + 8, heading_y_fig), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

            # self.get_logger().info(f"Heading min dist: {heading_min_distance:.2f} m at angle {math.degrees(heading_min_angle):.2f}°")
        else:
            self.get_logger().warn("No valid LiDAR data in heading direction")

        twist = Twist()


        # ----- Moving Close -----
        if not self.came_closer:
            if heading_min_distance is not None:

                # On first execution, record initial pose
                if self.first:
                    self.first_theta = self.current_pose.theta
                    self.first = False

                # Lateral drift correction
                Kp_ang = 1.5
                ang_err = 0.0

                # Normalize theta to range [-pi, pi]
                def normalize_angle(angle):
                    return (angle + math.pi) % (2 * math.pi) - math.pi

                # If facing near 0 or pi, correct Y drift
                if abs(self.first_theta % math.pi) < 0.05:
                    # Nearest multiple of pi
                    nearest_pi = round(self.first_theta / math.pi) * math.pi
                    ang_err = normalize_angle(nearest_pi - self.current_pose.theta)

                # If facing near pi/2 or 3pi/2, correct X drift
                elif abs((self.first_theta - math.pi / 2) % math.pi) < 0.05:
                    # Nearest multiple of pi/2
                    nearest_half_pi = round(self.first_theta / (math.pi / 2)) * (math.pi / 2)
                    ang_err = normalize_angle(nearest_half_pi - self.current_pose.theta)

                # General case: no drift correction
                else:
                    ang_err = 0.0

                twist.angular.z = float(np.clip(Kp_ang * ang_err, -self.calib_vro_max, self.calib_vro_max))

                # Linear velocity controller
                c = 10.0
                epsilon = 0.2
                tolerance = 0.04
                Kp = (1 / heading_min_distance) * (c / (heading_min_distance ** 2 + epsilon))
                twist.linear.x = float(np.clip(Kp * (heading_min_distance - 0.5), -self.vlin_max , self.vlin_max ))

                self.publisher.publish(twist)
                self.get_logger().info("Moving close!")

                if (heading_min_distance - 0.5) <= tolerance:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    self.publisher.publish(twist)
                    if abs(self.first_theta % math.pi) < 0.05:
                        nearest = round(self.first_theta / math.pi) * math.pi
                        ang_err = normalize_angle(nearest - self.current_pose.theta)
                    elif abs((self.first_theta - math.pi/2) % math.pi) < 0.05:
                        nearest = round(self.first_theta / (math.pi/2)) * (math.pi/2)
                        ang_err = normalize_angle(nearest - self.current_pose.theta)
                    else:
                        ang_err = 0.0

                    # apply same deadband + zeroing
                    raw_ang = Kp_ang * ang_err
                    if abs(ang_err) <= 0.01:
                        twist.angular.z = 0.0
                    else:
                        twist.angular.z = float(np.clip(raw_ang, -self.calib_vro_max, self.calib_vro_max))

                    self.get_logger().info("Came closerrrrrrrrrr!!")
                    self.publisher.publish(twist)
                    
                    self.came_closer = True
                    self.first = True  # Reset for the next phase

                # self.publisher.publish(twist)
                self.theta_facing = self.current_pose.theta
                self.last_class = self.current_class

                # cv2.imshow("LiDAR Visualization", img)
                if cv2.waitKey(1) & 0xFF == 27:  # ESC key
                    self.destroy_node()
                    cv2.destroyAllWindows()
                return

            else:
                self.get_logger().warn("Heading distance not available for moving close")
                return

        # # ----- Moving Close -----
        # if not self.came_closer:
        #     if heading_min_distance is not None:

        #         # On first execution, record initial pose
        #         if self.first:
        #             self.first_theta = self.current_pose.theta
        #             self.first = False

        #         # === approach‐phase lateral drift correction ===
        #         Kp_ang   = 1.0
        #         tol_zero = 0.01   # fine zeroing threshold

        #         # Normalize theta to [-π,π]
        #         def normalize_angle(angle):
        #             return (angle + math.pi) % (2*math.pi) - math.pi

        #         # compute ang_err as before…
        #         if abs(self.first_theta % math.pi) < 0.05:
        #             nearest = round(self.first_theta / math.pi) * math.pi
        #             ang_err = normalize_angle(nearest - self.current_pose.theta)
        #         elif abs((self.first_theta - math.pi/2) % math.pi) < 0.05:
        #             nearest = round(self.first_theta / (math.pi/2)) * (math.pi/2)
        #             ang_err = normalize_angle(nearest - self.current_pose.theta)
        #         else:
        #             ang_err = 0.0

        #         # apply deadband + zeroing
        #         raw_ang = Kp_ang * ang_err
        #         twist.angular.z = float(np.clip(raw_ang, -1.0, 1.0))

        #         # === linear controller ===
        #         c = 10.0; epsilon = 0.2; tolerance = 0.04
        #         Kp = (1/heading_min_distance)*(c/(heading_min_distance**2 + epsilon))
        #         twist.linear.x = float(np.clip(Kp*(heading_min_distance - 0.5),
        #                                     -self.vlin_max, self.vlin_max))

        #         self.publisher.publish(twist)
        #         self.get_logger().info("Moving close!")

        #         # === stop‐and‐align ===
        #         if (heading_min_distance - 0.5) <= tolerance:
        #             twist.linear.x = 0.0
        #             self.publisher.publish(twist)
        #             # recompute ang_err exactly as above…
        #             if abs(self.first_theta % math.pi) < 0.05:
        #                 nearest = round(self.first_theta / math.pi) * math.pi
        #                 ang_err = normalize_angle(nearest - self.current_pose.theta)
        #             elif abs((self.first_theta - math.pi/2) % math.pi) < 0.05:
        #                 nearest = round(self.first_theta / (math.pi/2)) * (math.pi/2)
        #                 ang_err = normalize_angle(nearest - self.current_pose.theta)
        #             else:
        #                 ang_err = 0.0

        #             # apply same deadband + zeroing
        #             raw_ang = Kp_ang * ang_err
        #             if abs(ang_err) <= tol_zero:
        #                 twist.angular.z = 0.0
        #             else:
        #                 twist.angular.z = float(np.clip(raw_ang, -1.0, 1.0))

        #             self.get_logger().info("Came closerrrrrrrrrr!!")
        #             self.publisher.publish(twist)
        #             self.came_closer = True
        #             self.first = True  # Reset for the next phase
        #         self.theta_facing = self.current_pose.theta
        #         self.last_class = self.current_class
        #     else:
        #         self.get_logger().warn("Heading distance not available for moving close")
        #         return

        #         # publish whatever final twist we have (approach or stopped+aligned)
        #         # self.publisher.publish(twist)
        #         # … rest unchanged …



        # ----- Control Logic -----

        self.get_logger().info(str(self.last_class))

        
        if self.last_class == 0:
            if heading_min_distance is not None:
                # Target: 90° to the LEFT of where the bot was originally facing
                target_yaw = self.theta_facing + math.pi/2

                # Get current yaw
                theta_odom = self.current_pose.theta  # Make sure this is being updated by pose_callback!

                # Compute signed angular error and wrap it to [–π, +π]
                error = (target_yaw - theta_odom + math.pi) % (2*math.pi) - math.pi

                # self.get_logger().info(f"target_yaw:   {target_yaw:.2f}")
                # self.get_logger().info(f"theta_odom:   {theta_odom:.2f}")
                # self.get_logger().info(f"angle error:  {error:.2f}")

                # P-control for angular velocity
                
                twist.angular.z = max(min(self.Kp_control_turn * error, self.vro_max), -self.vro_max)

                # Stop turning if within tolerance
                if abs(error) <= 0.01:
                    twist.angular.z = 0.0
                    self.publisher.publish(twist)
                    self.came_closer = False
                    self.get_logger().info("Turned left")
                else:
                    self.publisher.publish(twist)
            else:
                self.get_logger().warn("Error 0 :( No heading distance")

        elif self.last_class == 1:
            if heading_min_distance is not None:
                # Target: 90° to the LEFT of where the bot was originally facing
                target_yaw = self.theta_facing + math.pi/2

                # Get current yaw
                theta_odom = self.current_pose.theta  # Make sure this is being updated by pose_callback!

                # Compute signed angular error and wrap it to [–π, +π]
                error = (target_yaw - theta_odom + math.pi) % (2*math.pi) - math.pi

                # self.get_logger().info(f"target_yaw:   {target_yaw:.2f}")
                # self.get_logger().info(f"theta_odom:   {theta_odom:.2f}")
                # self.get_logger().info(f"angle error:  {error:.2f}")

                # P-control for angular velocity
                
                twist.angular.z = max(min(self.Kp_control_turn * error, self.vro_max), -self.vro_max)

                # Stop turning if within tolerance
                if abs(error) <= 0.01:
                    twist.angular.z = 0.0
                    self.publisher.publish(twist)
                    self.came_closer = False
                    self.get_logger().info("Turned left")
                else:
                    self.publisher.publish(twist)
            else:
                self.get_logger().warn("Error 1 :( No heading distance")

        elif self.last_class == 2:
            if heading_min_distance is not None:
                # 1) Compute the absolute target yaw: 90° to the right of initial facing
                target_yaw = self.theta_facing - math.pi/2
                # 2) Normalize into [–π, +π]
                theta_odom = self.current_pose.theta
                error = (target_yaw - theta_odom + math.pi) % (2*math.pi) - math.pi

                # self.get_logger().info(f"target_yaw:   {target_yaw:.2f}")
                # self.get_logger().info(f"theta_odom:   {theta_odom:.2f}")
                # self.get_logger().info(f"angle error:  {error:.2f}")

                # 3) P‐control, clamped to ±1.5 rad/s
              
                twist.angular.z = max(min(self.Kp_control_turn * error, self.vro_max), -self.vro_max)

                # 4) When within tolerance, stop and reset for the next detection
                if abs(error) <= 0.01:
                    twist.angular.z = 0.0
                    self.publisher.publish(twist)
                    self.came_closer = False
                    self.get_logger().info("Turned right ")
                else:
                    self.publisher.publish(twist)
            else:
                self.get_logger().warn("Error 2: no heading distance :( ")               

        elif self.last_class == 3:
            if heading_min_distance is not None:
                # 1) Compute the absolute target yaw: 180° from initial facing
                target_yaw = self.theta_facing + math.pi

                # 2) Get current yaw (now kept up‑to‑date by pose_callback)
                theta_odom = self.current_pose.theta

                # 3) Compute signed error wrapped to [–π, +π]
                error = (target_yaw - theta_odom + math.pi) % (2*math.pi) - math.pi

                # 4) P‑control on angular velocity
                twist.angular.z = max(
                    min(self.Kp_control_turn * error, self.vro_max),
                    -self.vro_max
                )

                # 5) If within tolerance, stop
                if abs(error) <= 0.01:
                    twist.angular.z = 0.0
                    self.publisher.publish(twist)
                    self.came_closer = False
                    self.get_logger().info("Turned 180°")
                else:
                    self.publisher.publish(twist)
            else:
                self.get_logger().warn("Error 3: no heading distance!")

        elif self.last_class == 4:
            if heading_min_distance is not None:
                # 1) Compute the absolute target yaw: 180° from initial facing
                target_yaw = self.theta_facing + math.pi

                # 2) Get current yaw (now kept up‑to‑date by pose_callback)
                theta_odom = self.current_pose.theta

                # 3) Compute signed error wrapped to [–π, +π]
                error = (target_yaw - theta_odom + math.pi) % (2*math.pi) - math.pi

                # 4) P‑control on angular velocity
                twist.angular.z = max(
                    min(self.Kp_control_turn * error, self.vro_max),
                    -self.vro_max
                )

                # 5) If within tolerance, stop
                if abs(error) <= 0.01:
                    twist.angular.z = 0.0
                    self.publisher.publish(twist)
                    self.came_closer = False
                    self.get_logger().info("Turned 180°")
                else:
                    self.publisher.publish(twist)
            else:
                self.get_logger().warn("Error 3: no heading distance!")

        elif self.last_class == 5:
            pass
        else:
            self.get_logger().warn(f"Unknown class: {self.last_class}")

        # Display the LiDAR visualization image.
        # cv2.imshow("LiDAR Visualization", img)
        if cv2.waitKey(1) & 0xFF == 27:  # Press 'Esc' to exit.
            self.destroy_node()
            cv2.destroyAllWindows()

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