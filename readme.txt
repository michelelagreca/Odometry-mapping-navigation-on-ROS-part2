# Mapping and Navigation for Autonomous Shuttle
The project focuses on map creation and autonomous navigation using multiple sensors on a skid steering robot. The project is implemented in ROS (Robot Operating System) and utilizes bag files for data input.

## Implementation

### Requirements
- ROS (Robot Operating System) installed
- C++ programming environment for ROS

### ROS Bag File
To use the ROS bag file, follow these steps:
1. Place the bag file in a suitable directory.
2. Open a terminal and navigate to the directory where the bag file is located.
3. Execute the following command to play the bag file:
```
rosbag play --clock -l <bag_file_name>.bag
```
Replace `<bag_file_name>` with the actual name of your bag file.

The bag files contain the following data topics:

- /scan: Data from a single plane scanner.
- /t265/odom: Odometry data.
- /velodyne_points: Data from a 3D laser.

## Problem Description
The project addresses the following key objectives:

1.Map Creation: Generate maps of the environment using data from sensors, including a single plane scanner, odometry, and 3D laser.
2.Autonomous Navigation: Develop a navigation system that allows the robot to move autonomously based on predefined waypoints.
3.Multiple Sensors: Utilize data from various sensors, such as the single plane scanner and 3D laser, to enhance mapping and navigation capabilities.
4.Skid Steering Robot: Implement the project on a skid steering robot with a specific footprint of 0.6m x 0.4m.
