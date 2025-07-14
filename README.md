# TurtleBot4 Perception and Object Tracking using ROS 2 & OpenCV

This project showcases a camera-based perception and control system for a mobile robot using ROS 2 and OpenCV. It involves two major components:

- **Task 4**: Object detection from a video stream and bounding box localization.
- **Task 5**: Real-time tracking of a red ball in Gazebo simulation and autonomous control of a TurtleBot4 using a PID controller.

The system detects visual input from a camera, extracts useful object features, and translates this perception into actionable motion commands for the robot.


## Tech Stack

- **ROS 2 (Humble)** – Robot framework for message passing and control  
- **Python 3** – All ROS 2 nodes are implemented in Python  
- **OpenCV 4.x** – Image processing and object detection  
- **cv_bridge** – ROS-to-OpenCV image conversion  
- **Gazebo** – Robot simulation environment  
- **vision_msgs**, **sensor_msgs**, **geometry_msgs** – Standard ROS 2 message types  



## Task 4: Object Detection from Video

- Publishes image frames from a pre-recorded `.avi` video using a ROS 2 publisher node. (The Object_Detection.avi was used to test)
- Processes each frame using OpenCV to detect an object using color segmentation and contour extraction.
- Calculates the object's bounding box, centroid coordinates, width, and height.
- Publishes the bounding box data using the `vision_msgs/msg/BoundingBox2D` message.
- Displays a live video window with the bounding box drawn.



## Task 5: Red Ball Tracking and Robot Control

- Subscribes to the robot's camera feed in simulation.
- Detects and tracks a red ball in real-time using HSV-based color masking.
- Computes ball position and distance to adjust robot velocity.
- Implements a PID controller to generate smooth, responsive motion:
  - Moves toward the ball when it’s far.
  - Stops or moves away when it’s too close.
  - Rotates to keep the ball centered in view.
- Publishes movement commands using `geometry_msgs/msg/Twist`.



## Demo
The video demo of the red ball tracker can be found [HERE](https://drive.google.com/file/d/1jmZindsVFnv2Bq90zbS64D2Jhg_fRj43/view?usp=drive_link)



## Future Enhancements

1. **Deploy on Real TurtleBot4 Hardware**  
   Extend the current simulation-based system to run on an actual TurtleBot4 robot. This would involve handling real-world camera input, lighting variability, and real-time ROS 2 node performance.

2. **Depth Estimation and 3D Tracking**  
   Integrate an RGB-D sensor (e.g., Intel RealSense) to estimate the object's distance in real-world units, enabling more accurate velocity control and richer robot behavior.

3. **Intelligent Search Behavior**  
   Implement a search strategy for when the ball is lost such as a spiral motion or SLAM-based visual memory, instead of remaining stationary or rotating in place.



## Course  
- **Purdue University** – ME 59700: Autonomous Systems Course 
