#!/usr/bin/env python3
""" 
go_to_goal.py

Author: Sai Vikas Satyanarayana, Keerthi Panguluri
Date: 3/24/25

 """
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D, Twist, Point
from std_msgs.msg import Float32 
from sensor_msgs.msg import LaserScan
import numpy as np
import math
from enum import Enum
import time


class State(Enum):
    GTG = 1  # Go to Goal
    AO = 2   # Avoid Obstacle
    FW_C = 3  # Follow Wall Clockwise
    FW_CC = 4  # Follow Wall Counter-Clockwise

class AdvancedNavigationController(Node):
    def __init__(self):
        super().__init__('advanced_navigation_controller')
        self.navigation_complete = False
        
        # Parameters
        self.d = 0.4  # Safety distance
        self.epsilon = 0.2 #0.17
        self.look_ahead = 0.17 #0.1
        self.v0 = 0.4
        self.alpha = 5000.0 #3000.0
        self.max_turn = 0.65 #1.2
        self.a = 500.0 #100.0
        self.c = 10.0
        self.epsilon_ao = 0.05
        self.vmax = 0.09
        
        # gains
        self.K_gtg = 1.0  # Gain for Go-to-Goal
        self.K_ao = 0.001   # gain for Avoid-Obstacle
        self.K_fw = 0.5   # Gain for Wall-Following

        # Max gains
        # self.K_gtg_max = 1.5  # Maximum gain for Go-to-Goal
        # self.K_ao_max = 2.0   # Maximum gain for Avoid-Obstacle
        self.u_ao_x = 0.0
        self.u_ao_y = 0.0
        self.ugt_x = 0.0
        self.ugt_y = 0.0

        # State variables
        self.current_pose = Pose2D()
        self.laser_data = None
        self.goals = [(1.5, 0.001), (1.5, 1.4), (0.0, 1.4)]  # Add more goals as needed
        # self.goals = [(-3.0,0.0)] 
        self.goal_index = 0
        self.goal_position = self.goals[self.goal_index]
        self.goal_radii = [0.03, 0.03, 0.03]
        # self.goal_radii = [0.1]
        self.state = State.GTG
        self.tau = self.get_clock().now()
        self.dist_at_tau = float('inf')
        self.obs_x = 0.0
        self.obs_y = 0.0

        
        # Subscribers and Publishers
        self.create_subscription(Pose2D, '/transferred_odom', self.pose_callback, 10)
        self.create_subscription(LaserScan, '/lidar_range', self.lidar_callback, 10)
        self.create_subscription(Float32, '/obstacle_distance', self.distance_callback, 10)
        self.create_subscription(Point, '/obstacle_coords',self.obs_coord_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.u_gtg_pub = self.create_publisher(Point, '/u_gtg', 10)
        self.u_ao_pub = self.create_publisher(Point, '/u_ao', 10)
        self.u_pub = self.create_publisher(Point, '/u', 10)
        
        self.timer = self.create_timer(0.1, self.control_loop)

    def pose_callback(self, msg):
        self.current_pose = msg

    def lidar_callback(self, msg):
        self.laser_data = msg.ranges

    def distance_callback(self,msg):
        self.distance_to_obstacle = msg.data

    def obs_coord_callback(self,msg):
        self.obs_x = msg.x
        self.obs_y = msg.y

    def calculate_u_gtg(self):
        dx = self.goal_position[0] - self.current_pose.x
        dy = self.goal_position[1] - self.current_pose.y
        norm = math.hypot(dx, dy)
        
        # Adaptive gain for Go-to-Goal (reduces as robot nears goal)
        self.K_gtg = self.v0 * (1 - math.e**(-self.a*norm*norm)) / norm
        # self.K_gtg = np.clip(self.K_gtg, 0, self.v0)
        # self.get_logger().info(f'norm: {norm}')
        self.ugt_x = self.K_gtg * dx
        self.ugt_y = self.K_gtg * dy
        
        return (self.ugt_x, self.ugt_y) if norm > 0 else (0,0)

    def calculate_u_ao(self):
        if not self.laser_data:
            return (0,0)
        
        # min_dist = min(self.laser_data) #this needs to be changed by narrowing the FOV and picking the min(mean,median) of the data.. means changing the get_object_range
        # min_idx = self.laser_data.index(min_dist)
        # angle = (min_idx / len(self.laser_data)) * 2 * math.pi
        # get the obstacle coordinates using lidar data and find norm = x_o - x, y_o - y
        # Get the obstacle coordinates in global frame
        dx = -self.obs_x
        dy = -self.obs_y
        norm = math.hypot(dx, dy)
        

        # self.K_ao = np.clip((1/norm)*(self.c/(norm**2 + self.epsilon_ao)), -50.0, 50.0)
        self.K_ao = (1/norm)*(self.c/(norm**2 + self.epsilon_ao))
        self.get_logger().info(f'K_ao: {self.K_ao}')
        
        # # Adaptive gain with max limit to prevent instability
        # self.K_ao = min(self.K_ao_max, 1 / (max(min_dist, 0.2) ** 2 + self.epsilon))

        
        # Calculate perpendicular avoidance direction
        # avoid_angle = angle + math.pi
        # return (self.K_ao * math.cos(avoid_angle), self.K_ao * math.sin(avoid_angle))
        self.u_ao_x = self.K_ao*dx
        self.u_ao_y = self.K_ao*dy

        theta_dum =  self.current_pose.theta
        R_dum = np.array([[math.cos(theta_dum), -math.sin(theta_dum)],
                    [math.sin(theta_dum), math.cos(theta_dum)]])
        u_dum = [self.u_ao_x, self.u_ao_y]
        u_dum = R_dum @ np.array(u_dum)
        self.u_ao_x = u_dum[0]
        self.u_ao_y = u_dum[1]
        return (self.u_ao_x, self.u_ao_y)

    def calculate_u_fw_c(self):
        theta = -np.pi/2.0 
        u_fw_c_x = self.alpha*(math.cos(theta)*self.u_ao_x - math.sin(theta)*self.u_ao_y)
        u_fw_c_y = self.alpha*(math.sin(theta)*self.u_ao_x + math.cos(theta)*self.u_ao_y)
        # u_fw_c_x = 0.0
        # u_fw_c_y = 0.0
        return (u_fw_c_x, u_fw_c_y)

    def calculate_u_fw_cc(self):
        theta = np.pi/2.0 
        u_fw_cc_x = self.alpha*(math.cos(theta)*self.u_ao_x - math.sin(theta)*self.u_ao_y)
        u_fw_cc_y = self.alpha*(math.sin(theta)*self.u_ao_x + math.cos(theta)*self.u_ao_y)
        # u_fw_cc_x = 0.0
        # u_fw_cc_y = 0.0
        return (u_fw_cc_x, u_fw_cc_y)

    def distance_to_goal(self):
        return math.hypot(self.goal_position[0] - self.current_pose.x,
                         self.goal_position[1] - self.current_pose.y)

    def control_loop(self):
        if self.laser_data is None or self.current_pose is None:
            return
        
        # Calculate all potential direction vectors
        u_gtg = self.calculate_u_gtg()
        u_ao = self.calculate_u_ao()
        u_fw_c = self.calculate_u_fw_c()
        u_fw_cc = self.calculate_u_fw_cc()

        u_gtg_msg = Point()
        u_gtg_msg.x = float(self.ugt_x)
        u_gtg_msg.y = float(self.ugt_y)
        u_gtg_msg.z = float(0)
        self.u_gtg_pub.publish(u_gtg_msg)

        u_ao_msg = Point()
        u_ao_msg.x = float(self.u_ao_x)
        u_ao_msg.y = float(self.u_ao_y)
        u_ao_msg.z = float(0)
        self.u_ao_pub.publish(u_ao_msg)
        
        # Calculate current distances
        dist_to_goal = self.distance_to_goal()
        dist_to_obs = self.distance_to_obstacle
        
        # State transitions
        if self.state == State.GTG:
            self.get_logger().info('GTG')
            if dist_to_obs <= self.d:
                dot_c = np.dot(u_gtg, u_fw_c)
                dot_cc = np.dot(u_gtg, u_fw_cc)
                if dot_c > 0:
                    self.state = State.FW_C
                    self.tau = self.get_clock().now()
                    self.dist_at_tau = dist_to_goal
                elif dot_cc > 0:
                    # self.state = State.FW_CC
                    self.state = State.FW_C
                    self.tau = self.get_clock().now()
                    self.dist_at_tau = dist_to_goal
                else:
                    self.get_logger().info('Error GTG!!!')
                # elif dist_to_obs < self.d - self.epsilon:
                #     self.state = State.AO
                    
        elif self.state == State.AO:
            self.get_logger().info('AO')
            if dist_to_obs > self.d:
                dot_c = np.dot(u_gtg, u_fw_c)
                dot_cc = np.dot(u_gtg, u_fw_cc)
                if dot_c > 0:
                    self.state = State.FW_C
                    self.tau = self.get_clock().now()
                    self.dist_at_tau = dist_to_goal
                elif dot_cc > 0:
                    # self.state = State.FW_CC
                    self.state = State.FW_C
                    self.tau = self.get_clock().now()
                    self.dist_at_tau = dist_to_goal
                else:
                    self.get_logger().info('Error AO!!!')
                    
        elif self.state == State.FW_C:
            self.get_logger().info('FW_C')
            current_time = self.get_clock().now()
            if (np.dot(u_gtg, u_ao) > 0 and ((self.distance_to_goal() < self.dist_at_tau))) or dist_to_obs >= self.d + self.epsilon:
                self.state = State.GTG
            elif dist_to_obs < self.d - self.epsilon:
                self.state = State.AO
                
        elif self.state == State.FW_CC:
            self.get_logger().info('FW_CC')
            current_time = self.get_clock().now()
            if (np.dot(u_gtg, u_ao) > 0 and ((self.distance_to_goal() < self.dist_at_tau))) or dist_to_obs >= self.d + self.epsilon:
                self.state = State.GTG
            elif dist_to_obs < self.d - self.epsilon:
                self.state = State.AO
                
        # Calculate control based on current state
        if self.state == State.GTG:
            u = u_gtg
        elif self.state == State.AO:
            u = u_ao
        elif self.state == State.FW_C:
            u = u_fw_c
        elif self.state == State.FW_CC:
            u = u_fw_cc
            
        # Convert direction to velocity commands (Single Integrator)
        theta = self.current_pose.theta
        R = np.array([[math.cos(theta), math.sin(theta)],
                     [-math.sin(theta), math.cos(theta)]])
        body_vel = R @ np.array(u)

        u_msg = Point()
        u_msg.x = float(u[0])#/math.hypot(u[0], u[1]))
        u_msg.y = float(u[1])#/math.hypot(u[0], u[1]))
        u_msg.z = float(0)
        # self.get_logger().info(f'ux :{u[0]}')
        # self.get_logger().info(f'uy :{u[1]}')
        self.u_pub.publish(u_msg)
        
        v = body_vel[0] 
        # v = 0.05
        w = body_vel[1] / self.look_ahead

        
        # Publish commands
        cmd_vel = Twist()
        cmd_vel.linear.x = np.clip(v, -self.vmax, self.vmax)
        cmd_vel.angular.z = np.clip(w, -self.max_turn, self.max_turn)
        # self.get_logger().info(f'Lin Vel: {cmd_vel.linear.x}, Ang Vel: {cmd_vel.angular.z}')
        self.cmd_vel_pub.publish(cmd_vel)

        if self.distance_to_goal() < self.goal_radii[self.goal_index]:  # Check if goal is reached
            self.get_logger().info(f'Reached goal {self.goal_index}, waiting 10 seconds')
            self.goal_index += 1
            stop_cmd = Twist()
            stop_cmd.linear.x = 0.0
            stop_cmd.angular.z = 0.0
            self.cmd_vel_pub.publish(stop_cmd)
            if self.goal_index >= len(self.goals):  # Stop if all goals are reached
                self.get_logger().info('All goals reached! Stopping.')
                self.navigation_complete = True  # Set flag to stop further execution
                self.destroy_node()
                return
            self.goal_position = self.goals[self.goal_index]  # Update to next goal
            self.get_logger().info(f'Next goal: {self.goal_position}')
            time.sleep(1)
            return  # Skip movement for this loop iteration

def main(args=None):
    rclpy.init(args=args)
    controller = AdvancedNavigationController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()