# Step 1 – Object Detection

In this first step, the goal was to get the robot to actually see and recognize an object in real time. Basically, I wanted the robot to know where a specific object is so that in the next steps it can rotate toward it, follow it, or navigate around it.

To do this, I wrote a Python node called `find_object.py` using **OpenCV**. The idea is simple: click on the object in the live camera feed, and the code figures out its color range in HSV. The robot (or your laptop webcam) then tracks that color, detects the object in each frame, and draws a bounding box with a marker at its center. It also prints out the pixel coordinates so you know exactly where the object is in the frame.

For hardware, I mainly used a **TurtleBot3** with its camera, but you can test it on a laptop webcam too. Using ROS2 made it easy to create the node and handle communication, while OpenCV and NumPy did the heavy lifting for image processing and object tracking.

Step 1 is all about perception — letting the robot see and understand its environment. The code from this step is reusable and sets the stage for the next steps, like rotating toward the object and eventually chasing it.
