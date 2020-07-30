# RoboND-HomeServiceRobot

This is the project repository of the final project of udacity robotics software nano degree. The project is developed to simulate virtual turtle bot in a virtual world setup. Turtlebot moves to predefined pick up zone and after that it moves to predefined drop zone. 

## Results

This project is fully developed with gazebo simulator. Output is given below

![alt text](output/output.gif)


## Implementation

### Shell scripts

Shell scripts are programmed to run multiple ros commands. Shell scripts are created as .sh file and change its execution type by
		chmod -x path_to_shell_script
And it can be run using commands 
  		./path_to_shell_script

### World and map

Virtual world, that was develpoed for previous projects, is also used for this project. It is a customized virtual world that is developed in Gazebo simulator.
Map file was developed using pgm_map_creator package 'https://github.com/udacity/pgm_map_creator'

### Robot

Turtlebot is used for this project. Package used ca be found in this repository. https://github.com/turtlebot/turtlebot_simulator

### Visualization

For visualizing results, default rviz configurations are implemented with simple modification. ROS package can be found under https://github.com/turtlebot/turtlebot_interactions

### Keyboard interations

For SLAM testing, keyboard navigation is required. turtlebot_teleop package is used for this purpose. Robot can be easily navigated using keys.

### SLAM algorithm

gmapping package is cloned for easy SLAM. Package url: https://github.com/ros-perception/slam_gmapping


## Shell Scripts

All shell scripts can be found under path 'src/scripts/'

test_slam.sh  -- For testing SLAM algorithm with keyboard manual navigation

test_navigation.sh  -- For testing navigation by setting 2D goals in rviz. 

pick_objects.sh -- for tesing pick objects node

add_markers.sh  -- for testing virtual object appearences

home_service.sh -- Complete scripts for testing this project

## Nodes

### pick_objects

pick_objects c++ nodes is developed to navigate robot to the pick zone and drop zone. This node sends navigation goal positon to the ROS. It is further controlled with AMCL algorithm. package: pick_objects

### add_markers

add_markers node creates virtual objects in the rviz for better visualization. A sphere shaped virtual object is displayed in the rviz visualization. For getting robot pose amcl_pose topic is subscribed. package: add_markers

## Setup workspace

		sudo apt-get update && apt-get upgrade
		sudo pat-get install xterm
		cd path_to_cloned_repository
		cd src/scripts
		chmod -x path_to_required_shell_script.sh
		./path_to_required_shell_script.sh

