
# Navvis Description Package 

## Description

This package builds the basic geometry for the Navvis robot. The package demonstrates the use of URDF and XACRO files for constructing geometry. The package 
includes a launch file that loads a URDF or XACRO file, instantiates ROS nodes for tracking joint angles of the robot's wheels, and visualizes the robot in
Gazebo.

## Dependencies

- Ubuntu 20.04
- ROS Noetic
- urdf
- xacro
- rviz
- sensor_msgs
- geometry_msgs
- joint_state_publisher_gui
- gazebo_plugins (if using the XACRO file to load the robot geometry)
- velodyne_description (if loading the XACRO file to load the robot geometry)

## Usage Instructions

To load the geometry with the URDF file and visualize the the robot with the joint state publisher gui, 
execute the launch file using roslaunch, the package name, and the launch file name. 

> roslaunch navvis_description display.launch 

To execute the package without the gui for specifying joint states, append the optional parameter use_gui:=false
To load the robot geometry via the XACRO file, append the optional parameter use_xacro:=true

> roslaunch navvis_description display.launch use_gui:=false use_xacro:=true

# Author
Nathan Mengers


