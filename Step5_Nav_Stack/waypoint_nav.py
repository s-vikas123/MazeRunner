#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action._navigate_to_pose import NavigateToPose_FeedbackMessage
import time

class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')
        self.publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.subscription = self.create_subscription(
            NavigateToPose_FeedbackMessage, '/navigate_to_pose/_action/feedback', self.feedback_callback, 20)
        # waypoints for the simulation
        self.waypoints = [
            (0.60, 1.1),
            (3.21, 0.83), 
            (4.10, 1.70)
        ] 
        # waypoints for the real world map
        # self.waypoints = [
        #     (1.11, -0.97), 
        #     (0.406, -0.78), 
        #     (0.031, 0.20)
        # ]
        self.current_waypoint_index = 0
        self.tolerance = 0.1  # Reduced tolerance for better accuracy
        time.sleep(2)  # Allow time for the publisher to be registered
        self.send_next_goal()

    def feedback_callback(self, msg):
        current_position = (msg.feedback.current_pose.pose.position.x, msg.feedback.current_pose.pose.position.y)
        target_x, target_y = self.waypoints[self.current_waypoint_index]
        distance = ((target_x - current_position[0])**2 + (target_y - current_position[1])**2) ** 0.5
        self.get_logger().info(f"Distance to waypoint {self.current_waypoint_index}: {distance}")
        
        if distance < self.tolerance:
            self.get_logger().info(f"Waypoint {self.current_waypoint_index + 1} reached.")
            self.current_waypoint_index += 1
            time.sleep(3)
            self.send_next_goal()

    def send_next_goal(self):
        if self.current_waypoint_index < len(self.waypoints):
            x, y = self.waypoints[self.current_waypoint_index]
            self.get_logger().info(f"Sending next goal to waypoint {self.current_waypoint_index + 1}: x={x}, y={y}")
            self.send_goal(x, y)
        else:
            self.get_logger().info("All waypoints reached.")

    def send_goal(self, x, y, z=0.0, orientation=[0.0, 0.0, 0.0, 1.0]):
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.get_clock().now().to_msg()

        goal.pose.position.x = float(x)
        goal.pose.position.y = float(y)
        goal.pose.position.z = float(z)

        goal.pose.orientation.x = float(orientation[0])
        goal.pose.orientation.y = float(orientation[1])
        goal.pose.orientation.z = float(orientation[2])
        goal.pose.orientation.w = float(orientation[3])

        self.publisher.publish(goal)
        self.get_logger().info(f"Goal sent to: x={x}, y={y}")


def main(args=None):
    rclpy.init(args=args)
    navigator = WaypointNavigator()
    rclpy.spin(navigator)
    navigator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
