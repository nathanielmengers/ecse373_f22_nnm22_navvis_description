# Navvis Description Package 

## Description

This package builds the basic geometry for the Navvis robot and visualizes it in Gazebo. The package demonstrates the use of URDF and XACRO files for constructing geometry. The launch file loads a URDF or XACRO file, instantiates ROS nodes for tracking joint angles of the robot's wheels, and runs Gazebo with a specified configuration.

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
- robot_state_publisher
- tf2_ros

## Installation Instructions
Assuming Ubuntu 20.04 and ROS Noetic have been installed, run the following code in the terminal:


> source /opt/ros/noetic/setup.bash
> mkdir catkin_ws
> cd catkin_ws
> catkin_make
> cd src

The package can be downloaded as a .zip file and extracted into the src folder. Alternatively, clone the repository from github:

> git clone https://github.com/nathanielmengers/ecse373_f22_nnm22_navvis_description.git

## Usage Instructions

To load the geometry with the provided urdf_from_xacro.urdf file and visualize the the robot with the joint state publisher gui, execute the launch file as below. In the default configuration below, the robot_state_publisher will also run.

> roslaunch navvis_description display.launch 

### Optional Arguments

- use_xacro: Defaults to false. Specify use_xacro to true if loading robot geometry from an XACRO file. Leave as false if using a URDF file.
> roslaunch navvis_description display.launch use_xacro:=true

- filename: Defaults to urdf_from_xacro.urdf if use_xacro = false or robot_description.xacro if use_xacro = true. To use another file in the package, specify as one of the following:

> roslaunch navvis_description display.launch filename:=<file_name.urdf>
> roslaunch navvis_description display.launch filename:=<file_name.xacro>


- file: Defaults to <package_path>/urdf/<filename>. If you would like to specify a URDF/XACRO file outside the package, specify the complete filepath including the directory, file name, and extension:

>roslaunch navvis_description display.launch file:=<file_location/file_name.urdf>

- use_robot_state_publisher: True by default. Executes the robot state publisher to track joint trajectories. If set to false, the wheel positions are set with the tf2_ros static_transform_publisher 

- use_gui: True by default. This allows the user to specify the joint angle of the wheels using the joint_state_publisher_gui. Setting to false executes joint_state_publisher instead. 
>roslaunch navvis_description display.launch use_gui:=false

- config_filename: Default configuration file is config.rviz in the contained config folder. The default shows only the robot with no sensing or map. The included config_with_lasers.rviz displays LIDAR data from three sensors on the robot. The included config_with_lasers_and_map.rviz additionally shows a 2D mapo of the environment. Note that users must provide their own map and LIDAR data. These are not included with the navvis_description package.

- config_file: Defaults to <package_path>/config/config.rviz. As with the geometry file, users may specify a configuration file in any other folder. 

>roslaunch navvis_description display.launch config_file:=<file_location/file_name.rviz>



## Author
Nathan Mengers


