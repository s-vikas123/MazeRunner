# Step 3 – Object Chasing with PID Control

The objective of Step 3 is to make the robot **chase a target object**, maintaining both a desired orientation and distance. Unlike Step 2, which only involved rotating toward the object, this step integrates **camera and LIDAR data** to achieve both angular and linear control.

## ROS 2 Package Structure

A new package called `krabbypatty_chase_object` was created. It contains four main nodes:

1. **`pick_color.py`**  
   - Subscribes to the compressed video stream (`/image_raw/compressed`).  
   - Allows selecting an HSV color by clicking on the object in the video feed.  
   - Publishes the selected color to `/picked_hsv`.  
   - OpenCV handles image display and color picking, while ROS2 manages communication.  

2. **`detect_object.py`**  
   - Subscribes to both the camera feed and the picked HSV color.  
   - Processes the frame to detect the object, calculates pixel coordinates, and converts them to **LIDAR-compatible angles**.  
   - Publishes the object’s coordinates in two topics: `/object/coordinates` (pixel coordinates) and `/object/coordinates_lidar_frame` (angular position).  
   - Includes filtering, morphological operations, and contour detection to ensure robust object recognition.  

3. **`get_object_range.py`**  
   - Subscribes to `/object/coordinates_lidar_frame` and the LIDAR scan (`/scan`).  
   - Computes the object’s **distance from the robot** using its angular position and LIDAR data.  
   - Publishes the result to `/object/range` as a `Point` message containing the angle and distance.  
   - Handles LIDAR noise by averaging multiple nearby scan points and ignoring invalid readings.  

4. **`chase_object.py`**  
   - Subscribes to both `/object/coordinates` and `/object/range`.  
   - Implements a high-level PID-like control:  
     - **Angular control:** rotates the robot to face the object.  
     - **Linear control:** moves forward or backward to maintain a desired distance.  
   - Publishes velocity commands to `cmd_vel`.  
   - Includes angular and linear tolerances to prevent jitter when the object is centered or within the target distance.  

## Integration of Camera and LIDAR

- The **camera** provides the object’s angular position in pixel coordinates. This is converted to an angle in the robot’s reference frame for compatibility with the LIDAR data.  
- The **LIDAR** provides depth information to accurately estimate the distance to the object.  
- Combining these sensors ensures reliable object tracking even when camera data alone is insufficient for depth estimation.  

## Control Logic

- Angular error is computed as the difference between the image center and the object’s horizontal position.  
- Linear error is computed as the difference between the object’s distance and a predefined desired distance (e.g., 0.35 m).  
- Both errors are multiplied by proportional gains (Kp) to produce smooth velocity commands.  
- Velocity commands are capped to maintain safe and stable movement.  

## Execution

- Nodes can run either on the TurtleBot3 or on an offboard computer.  
- Running computation on the robot minimizes network lag and ensures real-time performance.  
- QoS settings in ROS2 ensure reliable data transfer between nodes.  
- Launch files can start all nodes simultaneously for seamless execution during demos.  

Step 3 combines **perception, sensor fusion, and control**, enabling the robot to chase and follow a moving object while maintaining proper alignment and distance.
