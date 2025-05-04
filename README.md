# 🚜 Forklift Odometry (ETV216)

## 📖 Overview
The **Forklift Odometry Project** develops a ROS package to estimate the odometry of a **Jungheinrich ETV216 forklift** using CAN bus data and the robot’s kinematic model to publish accurate pose and velocity information.

## 📁 Package Content

- `cititruck_description`  
  - `launch/`: RViz config files  
  - `mesh/`: 3D models of the forklift (.dae format)  
  - `urdf/`: Xacro files for the forklift description  

- `launch/`: All ROS launch files  
- `rviz_configs/`: RViz configurations  
- `scripts/`:  
  - `Forklift_Kinematics_Odometry.py`: Final ROS node  
  - `Forklift_Kinematics_Odometry_1.py`: Development trials  

- `CMakeLists.txt`  
- `package.xml`

## ⚙️ Node Description

The `ForkliftOdometry` ROS node:
- Subscribes to CAN frames for wheel travel and steering angles
- Calculates linear and angular velocities
- Updates the forklift’s pose and orientation using kinematic equations
- Publishes odometry and path info as ROS messages
- Broadcasts transform between `odom` and `base_link`

## 🔧 Features

- 📡 CAN-based data input  
- 🔁 Real-time odometry calculation  
- 🧭 Path generation using PoseStamped messages  
- 🔄 Transform broadcasting with TF  
- ⚙️ Configurable parameters (e.g. wheelbase, frame ID)

## 🧩 Node Structure

- **Initialization**: Publishers/subscribers setup  
- **Frame Handling**: Parses CAN data and triggers calculations  
- **Odometry & Path Publishing**: Uses kinematics for motion estimation  
- **TF Broadcasting**: Keeps coordinate frames aligned  
- **Main Execution**: Runs the node at a defined frequency  

## 🎥 Demo
![Forklift Odometry Demo](https://github.com/AhmedHusseinElsayed19/Forklift_Odometry/assets/39027317/0c5ab961-6721-41e4-b615-03fc7fd233f8)
