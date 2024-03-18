# Forklift_Odometry

# Overview
The Forklift Odometry Project aims to develop a ROS (Robot Operating System) package for estimating the odometry of a Jungheinrich ETV216 forklift. This package utilizes CAN bus data which includes: the wheel travel and the steering angle with the robot's kinematic model to calculate and publish the robot's pose and velocity information. 

# Package Content:

1.cititruck_description : 
a. launch folder: all the RVIZ files are included in this folder.
b. mesh folder: all the .dae files that represent the 3D models of the forklift and its additional parts.
c. urdf folder: all the .xacro files and trials of the ETV216 forklift are included.

2.launch:
all the .launch files are presented here

3.rviz_configs

4.scripts: 
It has two files one for the main code of the ROS node and the second one includes all the trials that took place to reach the final target.

5.CMakeLists.txt

6.package.xml


# Description:

The ForkliftOdometry ROS node is responsible for estimating the odometry of a forklift robot based on CAN bus data and the robot's kinematic model. It subscribes to CAN frame messages and calculates the forklift's pose and velocity information. It also publishes the odometry and path information as ROS messages.

# Functionality and Features
Subscribes to CAN frame messages containing wheel travel and steering angle information.
Calculates the linear and angular velocities of the forklift based on the wheel travel and steering angle values.
Updates the position and orientation of the forklift using kinematic equations.
Publishes the odometry information as Odometry messages on the odom topic.
Publishes the path information as Path messages on the path topic.
Broadcasts the transform between the odom frame and the base_link frame.
Supports configurable parameters such as the wheelbase of the forklift and the frame ID for which to calculate odometry.

# Node Structure
The ForkliftOdometry node is implemented as a Python script (forklift_odometry.py) and consists of the following main components:

a. Initialization: The node is initialized, and necessary ROS publishers, subscribers, and variables are set up.
Frame Handling: The handle_frame method is called whenever a frame message is received. It extracts the wheel travel and steering angle values from the frame and calculates the odometry information.
b. Odometry Calculation: The handle_frame method calculates the linear and angular velocities, updates the position and orientation using kinematic equations, and publishes the odometry message.
c. Path Generation: The handle_frame method generates a path by creating a PoseStamped message for the current position and adding it to the Path message.
d. Transform Broadcasting: The handle_frame method broadcasts the transform between the Odom frame and the base_link frame.
e. Publishing: The publish_odometry and publish_path methods publish the odometry and path messages, respectively.
f. Main Execution: The run method is called in the main execution block, which runs the node at a specified rate.
