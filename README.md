Forklift_Odometry
Overview
The Forklift Odometry Project aims to develop a ROS (Robot Operating System) package for estimating the odometry of a Jungheinrich ETV216 forklift. This package utilizes CAN bus data, including wheel travel and steering angle, along with the robot's kinematic model to calculate and publish the robot's pose and velocity information.

Package Content
cititruck_description:

launch folder: Contains all the RVIZ files.
mesh folder: Includes the 3D models of the forklift and its additional parts in .dae format.
urdf folder: Contains the .xacro files and trials of the ETV216 forklift.
launch: Contains all the .launch files.

rviz_configs:

scripts:

forklift_odometry.py: The main code for the ROS node.
trials.py: Includes all the trials that took place to reach the final target.
CMakeLists.txt

package.xml

Description
The ForkliftOdometry ROS node is responsible for estimating the odometry of a forklift robot based on CAN bus data and the robot's kinematic model. It subscribes to CAN frame messages and calculates the forklift's pose and velocity information. It also publishes the odometry and path information as ROS messages.

Functionality and Features
Subscribes to CAN frame messages containing wheel travel and steering angle information.
Calculates the linear and angular velocities of the forklift based on the wheel travel and steering angle values.
Updates the position and orientation of the forklift using kinematic equations.
Publishes the odometry information as Odometry messages on the odom topic.
Publishes the path information as Path messages on the path topic.
Broadcasts the transform between the odom frame and the base_link frame.
Supports configurable parameters such as the wheelbase of the forklift and the frame ID for which to calculate odometry.
Node Structure
The ForkliftOdometry node is implemented as a Python script (forklift_odometry.py) and consists of the following main components:

Initialization: The node is initialized, and necessary ROS publishers, subscribers, and variables are set up.

Frame Handling: The handle_frame method is called whenever a frame message is received. It extracts the wheel travel and steering angle values from the frame and calculates the odometry information.

Odometry Calculation: The handle_frame method calculates the linear and angular velocities, updates the position and orientation using kinematic equations, and publishes the odometry message.

Path Generation: The handle_frame method generates a path by creating a PoseStamped message for the current position and adding it to the Path message.

Transform Broadcasting: The handle_frame method broadcasts the transform between the odom frame and the base_link frame.

Publishing: The publish_odometry and publish_path methods publish the odometry and path messages, respectively.

Main Execution: The run method is called in the main execution block, which runs the node at a specified rate.
