# Step 2 – Object Following (Rotate Robot)

After getting the robot to detect objects in Step 1, the next goal was to make it actually **turn toward the object**. The idea here was simple: the robot should rotate in place so the object stays centered in its camera view. No forward or backward movement yet – just turning.

To make this happen, I created a new ROS2 package called `team_object_follower`. Inside, I added three main nodes:

1. **`color_picking.py`** – This node lets you click on the object in the camera feed to select its color in HSV. The selected color is published to `/picked_hsv`. I used OpenCV for displaying the feed and picking the color, and ROS2 for publishing it.

2. **`find_object1.py`** – This subscribes to the camera feed (`/image_raw/compressed`) and the picked HSV color. It processes each frame to detect objects of that color, publishes the coordinates to `/object/coordinates`, and optionally publishes the processed image for debugging. OpenCV handles all the image processing, while ROS2 handles the messaging.

3. **`rotate_robot.py`** – This node subscribes to `/object/coordinates` and publishes velocity commands to `cmd_vel` so the robot rotates to center the object. I implemented a simple proportional controller so the rotation is smooth, and added a small tolerance to avoid jitter when the object is almost centered.

For hardware, I ran the nodes on a **TurtleBot3 Burger**, using its Raspberry Pi camera. To speed up development and debugging, I sometimes ran the image processing on my laptop, but the final setup runs everything on the robot itself. Gazebo simulation was also used for initial testing, especially to verify that the rotation logic works without worrying about Wi-Fi lag.

Some things I kept in mind:
- Network lag matters! Running image processing on the laptop adds delay, so the final demo had everything on the robot.
- QoS settings in ROS2 were important for reliable image streaming over Wi-Fi.
- Launch files helped me start multiple nodes at once without juggling several SSH terminals.

Step 2 is all about **perception + action** – detecting the object and making the robot rotate to track it. This sets the stage for full object following in Step 3, where we’ll combine rotation with forward movement.
