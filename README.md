# MazeRunner
From vision to navigation – a TurtleBot’s journey through mazes

This repository documents the step-by-step journey of building autonomy in a mobile robot as part of the Introduction to Robotics course. Each step builds on the previous one, gradually moving from simple perception to advanced navigation and maze-solving. The project integrates OpenCV, LiDAR, ROS2, and the Navigation Stack, demonstrating how perception, planning, and control come together in robotics. 

# Step 1 – Object Detection with OpenCV

In this step , we implemented a real-time object detection system based on color segmentation. The system allows the user to select a target color by clicking directly on the live video feed, dynamically updating the HSV range in OpenCV. Once a color is chosen, the robot’s camera highlights and tracks objects matching the specified color in real time.

# Step 2 – Rotate to Face Object

Building on the object detection system, this step enables the robot to detect the user-selected object in its camera feed, determine its relative position, and rotate until it is directly facing the object. The live video feed is streamed to the user’s computer, providing immediate visual feedback of the robot’s orientation adjustment.

# Step 3 – Object Chasing with Camera and LiDAR

This step combines vision and LiDAR sensing to achieve dynamic object following. The user selects the object color from the live feed (HSV format). The robot then:

Detects the object in its camera view

Estimates the object’s relative position and distance using camera–LiDAR fusion

Autonomously drives toward the object while maintaining a safe, predefined distance

This introduces multi-sensor integration for real-time perception and control.

# Step 4 – Go-to-Goal with Obstacle Avoidance

In this step, the robot navigates toward predefined goal points while avoiding unknown obstacles using LiDAR data. The robot dynamically adjusts its trajectory to safely reach the target, demonstrating reactive motion planning in cluttered environments.

# Step 5 – Autonomous Navigation with ROS2 Navigation Stack

This step integrates the ROS2 Navigation Stack (Nav2) for global path planning and autonomous navigation. The robot is tasked with reaching a sequence of global waypoints, both in simulation and on the real robot. This includes:

Map-based localization

Path planning

Obstacle avoidance

Goal-directed navigation

# Step 6 – Maze Navigation

The final step focuses on enabling the robot to autonomously navigate through a maze-like environment. The robot applies perception and navigation strategies to explore, avoid dead ends, and successfully reach the exit, showcasing integrated autonomy across sensing, planning, and control.
