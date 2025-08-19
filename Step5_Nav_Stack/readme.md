# Step 5 – Autonomous Navigation with ROS2 Nav2 Stack

The objective of Step 5 is to familiarize the robot with **mapping, localization, and path planning** capabilities of the ROS2 Navigation Stack (Nav2). The ultimate goal is to make a simulated and real TurtleBot3 autonomously navigate to a series of global waypoints. Unlike previous steps, this lab emphasizes **parameter tuning and integration** rather than implementing core algorithms from scratch.  

Official Nav2 documentation: [https://navigation.ros.org/index.html](https://navigation.ros.org/index.html)

---

## ROS 2 Package Structure and Nodes

### 1. Waypoint Navigator (`waypoint_navigator.py`)
- Publishes global navigation goals to the `/goal_pose` topic using `PoseStamped` messages.  
- Subscribes to `/navigate_to_pose/_action/feedback` using `NavigateToPose_FeedbackMessage` to track the robot’s progress.  
- Waypoints can be defined for both simulation and real-world maps.  
- Implements **tolerance checking** to determine when a waypoint is reached and automatically sends the next goal.  
- Uses a **timer delay** to ensure goals are published after the publisher is registered, providing smoother transitions between waypoints.

### 2. AMCL (Adaptive Monte Carlo Localization)
- Configured through `burger.yaml` parameters.  
- Parameters such as `alpha1-alpha5` tune odometry noise, while `max_particles` and `min_particles` control localization accuracy.  
- Particle filter uses laser scan data to localize the robot on the map with high reliability.  
- Supports beam skipping, likelihood fields, and various motion models for robust localization in both simulation and real environments.  

### 3. BT Navigator (Behavior Tree)
- Configured in Nav2 to handle **autonomous decision-making**, including replanning and recovery.  
- Plugins allow actions such as path following, obstacle avoidance, spinning, backing up, and waiting.  
- Groot monitoring enabled for visualizing execution flow of the behavior tree.  

### 4. Controller Server
- Implements **Regulated Pure Pursuit Controller** to follow planned paths.  
- Parameters tuned for the maze environment, including lookahead distances, linear velocities, and angular velocities.  
- Includes **collision detection** and velocity regulation to prevent unsafe maneuvers.  
- Uses progress and goal checkers to ensure accurate path execution.  

### 5. Costmaps
- **Local Costmap:** dynamic obstacle detection around the robot using LIDAR (`scan` topic), includes voxel, obstacle, and inflation layers.  
- **Global Costmap:** represents the environment for global planning, updated with static maps and LIDAR observations.  
- Parameters tuned for resolution, robot radius, inflation factors, and obstacle clearance.  

### 6. Map Server and Saver
- Provides static map for localization and path planning.  
- Allows saving maps after teleoperation or SLAM mapping.  

### 7. Planner Server
- Global planner computes paths to waypoints using either A* or Dijkstra algorithms.  
- Tolerance and unknown space handling can be tuned through configuration parameters.  

### 8. Recovery Server
- Handles unexpected situations like stuck robot or failed goal.  
- Includes recovery behaviors: `spin`, `backup`, and `wait`.  
- Ensures navigation continues robustly without manual intervention.

### 9. Robot State Publisher
- Publishes transforms (`tf`) of robot frames for accurate visualization and navigation.  

### 10. Waypoint Follower
- Executes a series of waypoints with optional **pause durations** at each waypoint.  
- Configurable plugin-based system for customizable behaviors at each waypoint.

---

## Workflow

1. **Map Creation**
   - Generate a map using teleoperation or SLAM in Gazebo or real environment.  
   - Save the map as a `.yaml` file and load it through the map server.  

2. **Localization**
   - Initialize the robot at a known starting pose.  
   - AMCL uses laser scans and odometry to localize the robot in the global map frame.  

3. **Path Planning**
   - Global planner computes the optimal path to the next waypoint.  
   - DWB or Pure Pursuit controllers generate velocity commands for safe navigation.  

4. **Navigation Execution**
   - Waypoint navigator publishes goals sequentially.  
   - BT Navigator handles path following, obstacle avoidance, and recovery actions.  
   - Feedback from `/navigate_to_pose/_action/feedback` allows monitoring progress and moving to next waypoint.  

5. **Tuning Parameters**
   - Costmap parameters define obstacle influence and inflation radius.  
   - Controller parameters define lookahead distance, speed, and collision response.  
   - AMCL particle filter parameters control localization precision.  
   - Adjusting planner and controller frequencies ensures smooth and real-time navigation.

---

## Hardware
- TurtleBot3 Burger  
- LIDAR sensor (Hokuyo or equivalent)  
- Wheel encoders for odometry  

---

## Software
- ROS2 Humble  
- Python 3 for node implementation  
- Nav2 Stack for navigation, planning, and behavior tree execution  
- Gazebo for simulation testing  
- RViz for visualization and debugging  

---

## Execution

- Launch all required Nav2 nodes using standard launch files, including map server, AMCL, planner, controller, and waypoint follower.  
- Start navigation in simulation or on the real robot.  
- Robot autonomously navigates through the defined series of global waypoints, handling obstacles and replanning as necessary.  
- Feedback logging allows real-time monitoring of distance to goals and waypoint completion.

---

Step 5 combines **mapping, localization, global and local planning, and autonomous waypoint navigation** into a fully integrated system using ROS2 Nav2. The robot can navigate complex environments in simulation and real-world conditions, making it suitable for practical autonomous robotics research and testing.
