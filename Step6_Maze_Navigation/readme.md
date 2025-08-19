# Step 6 – Maze Navigation with Sign Classification

The goal of this step is to make the TurtleBot autonomously navigate through a maze using both **LIDAR for obstacle avoidance** and **YOLOv8-based computer vision** for directional sign classification. The system combines perception, localization, and control so the robot can approach walls, read signs, and make decisions that guide it to the goal.

---

## ROS2 Package Structure

A dedicated ROS2 package was created with four main nodes working together:

### 1. **YOLO Classifier Node – `detect_direction_practical.py`**
- Runs a pre-trained **YOLOv8 classifier model** on live camera input.  
- Subscribes to `/simulated_camera/image_raw/compressed`.  
- Applies preprocessing (cropping and upscaling) to boost classification accuracy.  
- Performs inference on both raw and preprocessed images, then selects the result with higher confidence.  
- Publishes the detected sign class to the `/detected_class` topic as a `String`.  

### 2. **LIDAR Coordinates Node – `lidar_coordinates_node.py`**
- Provides utilities for transforming coordinates between **LIDAR** and **map** frames using TF2.  
- Handles quaternion-to-Euler conversions for working with angles.  
- Mainly used for debugging and verifying correct spatial transformations.  

### 3. **Navigation Control Node – `next_coord_direction_control.py`**
- Core node that drives the maze-solving behavior.  
- Subscribes to `/scan` (LIDAR data) and `/detected_class` (sign output).  
- Implements a **state machine** with two main phases:
  - **Approach Phase:** Move toward walls, maintain heading, and correct drift.  
  - **Action Phase:** Execute maneuvers (left, right, U-turn, stop) based on detected sign.  
- Publishes velocity commands to `/cmd_vel`.  
- Uses proportional control for smooth turns and consistent wall-following.  

### 4. **Odometry Publisher – `odometrypublisher.py`**
- Normalizes robot odometry so that every run starts from `(0,0)` with heading along the x-axis.  
- Subscribes to `/odom` and publishes transformed data to `/transferred_odom`.  
- Ensures a clean and consistent frame of reference for navigation.  

---

## Sign Categories

The system is trained to recognize six types of maze signs:

- **Class 0:** Empty wall → Turn left  
- **Class 1:** Left arrow → Turn left 90°  
- **Class 2:** Right arrow → Turn right 90°  
- **Class 3:** Do Not Enter → U-turn (180°)  
- **Class 4:** Stop sign → U-turn (180°)  
- **Class 5:** Goal marker → Stop navigation  

---

## Algorithm Workflow

### Navigation State Machine
1. **Approach Phase**  
   - Drive toward the nearest wall using LIDAR.  
   - Keep heading straight with drift correction.  
   - Stop at ~0.5 m from the wall for sign reading.  

2. **Classification Phase**  
   - Camera captures the wall.  
   - YOLOv8 processes the image and publishes the detected class.  

3. **Action Phase**  
   - Turn according to the detected sign (left, right, U-turn, or stop).  
   - Use proportional control to achieve precise angles.  
   - Resume approach phase until the next sign is encountered.  

### Vision Pipeline
1. Capture image from `/simulated_camera/image_raw/compressed`.  
2. Apply preprocessing: crop + upscale.  
3. Run YOLOv8 inference.  
4. Choose prediction with highest confidence.  
5. Publish class to `/detected_class`.  
