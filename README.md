# Radio_source_tracking
This is the github to the Project of Topics in Computer Science course
Introduction
The Radio Source Tracking Simulation project aims to simulate a radio frequency source and a radio receiver in a Gazebo environment. The receiver's goal is to detect and track the position of the radio source.

Features
Simulation of a radio frequency source.
Simulation of a radio receiver.
Tracking and localization algorithms.
Visualization of the tracking process in Gazebo.

Installation
1. Clone the repository:
git clone https://github.com/VuHoangNguyena1869879/Radio_source_tracking.git
cd Radio_source_tracking
2. Make sure ROS2 and Gazebo Harmonic are installed
3. Build the project
colcon build
source install/setup.bash
Usage
1. Launch the Gazebo simulation
ros2 launch rf_sensor simulation.launch.py


