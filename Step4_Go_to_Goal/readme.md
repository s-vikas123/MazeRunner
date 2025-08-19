# Step 4 – Navigation Through Waypoints with Obstacle Avoidance

The objective of Step 4 is to enable the robot **navigate through a sequence of predefined waypoints** while avoiding unknown obstacles dynamically introduced in its path. This step integrates **odometry, LIDAR, and waypoint management** to perform hybrid navigation.

## ROS 2 Package Structure

A new package called `TeamName_navigate_to_goal` was created. It contains several main nodes:

1. **`get_object_range.py`**  
   - Subscribes to the LIDAR scan topic (`/lidar_range`).  
   - Processes the 360° scan to identify obstacles and calculate the nearest point relative to the robot.  
   - Publishes the obstacle coordinates to `/obstacle_coords` and distance to `/obstacle_distance`.  
   - Implements filtering to ignore noise and irrelevant obstacles like walls or stray objects.  

2. **`go_to_goal.py`**  
   - Subscribes to the robot’s odometry (`/transferred_odom`) and the obstacle information from `get_object_range.py`.  
   - Reads waypoint coordinates either from a file or predefined list.  
   - Implements a **hybrid controller** with four behaviors (basically hardcoded **Dynamic Window Approach**):  
     - **Go-to-Goal (GTG):** Moves directly toward the current waypoint using adaptive gains.  
     - **Avoid Obstacle (AO):** Diverts the robot from obstacles using perpendicular repulsion vectors.  
     - **Wall-Following Clockwise (FW_C):** Circumnavigates obstacles when direct avoidance is blocked.  
     - **Wall-Following Counter-Clockwise (FW_CC):** Alternative path for circumnavigation if needed.  
   - Publishes velocity commands to `/cmd_vel` and debug vectors to `/u_gtg`, `/u_ao`, and `/u`.  
   - Ensures the robot stops within defined tolerances at each waypoint and proceeds sequentially to the next goal.  

3. **Odometry Initialization**  
   - A helper script adjusts the starting pose to (0,0) with heading along the x-axis.  
   - Ensures consistent starting conditions for multiple runs by applying offsets to the onboard odometry.

---

## Integration of Sensors and Control

- **Odometry** provides global position estimation using onboard encoders and dead reckoning.  
- **LIDAR** provides local obstacle detection in real time.  
- The hybrid controller combines these data sources to decide motion: GTG drives toward goals, AO repels from obstacles, and wall-following ensures safe navigation around obstacles.  
- Velocity commands are computed using a single-integrator model and transformed from global to robot body frame.

---

## Hardware Used
- TurtleBot3 robot platform  
- Onboard LIDAR for obstacle detection  
- Odometry sensors (wheel encoders)  

---

## Software and Tools
- ROS2 Humble for node communication and control  
- Python 3 for node implementation  
- Standard ROS2 message types: `Pose2D`, `Twist`, `Point`, `Float32`, `LaserScan`  
- RViz for visualization of robot pose and debug vectors  

---

## Execution

- Nodes can run either on the TurtleBot3 or on an offboard computer.  
- LIDAR and odometry data are continuously processed to compute control commands.  
- The robot navigates through waypoints while avoiding obstacles and stopping at each waypoint for the required duration.  
- Launch files can start all nodes simultaneously for smooth execution during demos.  

Step 4 combines **perception, hybrid control, and real-time navigation**, enabling the robot to safely reach multiple waypoints in an unknown environment while avoiding obstacles.
