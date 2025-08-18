# Step 2 – Object Following (Rotate Robot)

After detecting objects in Step 1, the next goal is to make the robot **turn toward the object**. The objective is for the robot to rotate in place so that the object stays centered in its camera view, without any forward or backward movement.

A new ROS2 package called `krabbypatty_object_follower` was created, containing three main nodes:

1. **`color_picking.py`** – Subscribes to the camera feed (`/image_raw/compressed`) and allows selecting an HSV color by clicking on the object in the video. The selected color is published to `/picked_hsv`. OpenCV is used for displaying the feed and picking the color, while ROS2 handles the messaging.

2. **`find_object1.py`** – Subscribes to the camera feed and the picked HSV color. Processes each frame to detect objects of that color, publishes the coordinates to `/object/coordinates`, and optionally publishes the processed image for debugging. OpenCV handles the image processing, and ROS2 manages topic communication.

3. **`rotate_robot.py`** – Subscribes to `/object/coordinates` and publishes velocity commands to `cmd_vel` to rotate the robot toward the object. A proportional controller is implemented for smooth rotation, with a small tolerance to avoid jitter when the object is almost centered.

The nodes are executed on a **TurtleBot3 Burger** using its Raspberry Pi camera. Image processing can be run offboard on a computer for debugging, but the final setup runs all nodes on the robot to minimize network lag. Gazebo simulation is also used for testing rotation logic without relying on Wi-Fi.

Key considerations:
- Network lag can affect responsiveness, so running processing directly on the robot ensures real-time performance.
- Quality of Service (QoS) settings in ROS2 help maintain reliable image streaming.
- Launch files allow starting multiple nodes simultaneously, reducing the need to manage several SSH terminals.

Step 2 combines **perception and action**: detecting an object and rotating the robot to keep it centered. This lays the foundation for full object following in later steps, where forward movement will be added.
