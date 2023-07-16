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

- `/scan`: Data from a single plane scanner.
- `/t265/odom`: Odometry data.
- `/velodyne_points`: Data from a 3D laser.

## Problem Description
The project addresses the following key objectives:

1. Map Creation: Generate maps of the environment using data from sensors, including a single plane scanner, odometry, and 3D laser.
2. Autonomous Navigation: Develop a navigation system that allows the robot to move autonomously based on predefined waypoints.
3. Multiple Sensors: Utilize data from various sensors, such as the single plane scanner and 3D laser, to enhance mapping and navigation capabilities.
4. Skid Steering Robot: Implement the project on a skid steering robot with a specific footprint of 0.6m x 0.4m.

## Project Structure
The project is organized into a ROS package called `second_project`. The package contains the following components:

### tf_publisher Node
- Name: `tf_publisher`
- File: `tf_publisher.cpp`

This node converts the odometry topic into a tf (transform) to establish the relationship between the world and the robot frames. It publishes the tf transformations required for mapping and navigation.

### Mapping
The mapping process involves converting 3D laser data to a 2D laser scanner format and creating maps of the environment.

#### Conversion of 3D LIDAR Data to 2D
- Node: `pointcloud_to_laserscan`
- File: `mapping_lidar.launch`

This node converts the 3D LIDAR data from `/velodyne_points` to a 2D laser scanner format and publishes the resulting data on the `/2d_lidar topic`. It sets appropriate parameters for the conversion process, such as minimum and maximum heights, angle increment, and range maximum.

#### Mapping with Laser Data
- Node: `slam_toolbox`
- File: `mapping_laser.launch`

This node utilizes the Slam Toolbox to perform mapping using the laser data obtained from the `/scan` topic. It loads the configuration parameters from `st_config_laser.yaml` and outputs the map of the environment. The launch file also starts the RViz visualization tool with a proper configuration file.

#### Mapping with 3D Laser Data (Converted to 2D)
- Node: `slam_toolbox`
- File: `mapping_lidar.launch`

This node uses the Slam Toolbox to create a map of the environment using the 3D laser data converted to 2D format. It loads the configuration parameters from `st_config_lidar.yaml` and generates the map. The launch file also starts RViz for visualization.

### Navigation
The navigation part of the project enables the robot to autonomously navigate based on predefined waypoints.

- Node: `navigation`
- File: `navigation.cpp`

This node reads a set of waypoints from a CSV file (`waypoints.csv`) and publishes them as goals for the robot to follow using the move_base package. The move_base node handles the movement to each goal and progresses to the next one once the current goal is reached. The action mechanism is utilized for communication between the navigation node and move_base.

### Launch Files
- File: `mapping_laser.launch`
  - Starts all the required nodes for laser mapping, including `tf_publisher`, `slam_toolbox`, and RViz. It also sets appropriate transformations for the laser data.
- File: `mapping_lidar.launch`
  - Starts all the required nodes for 3D lidar mapping, including `tf_publisher`, `pointcloud_to_laserscan`, `slam_toolbox`, and RViz. It also sets the necessary transformations.
- File: `robot_amcl.launch`
  - Launches the robot simulation in Stage, along with the navigation stack. It starts nodes such as `stageros`, `move_base`, `map_server`, `amcl`, and RViz. The initial robot position, map file, and other parameters can be configured in this launch file.
## Result Analysis
The project achieved the following results:

- `TF and Transformations`: The `tf_publisher` node establishes the required transformations between different frames, including odom, t265, base_footprint, base_link, velodyne, and laser. It sets up a transformation tree similar to the provided bag file, with additional static transformations for the missing frames.
- `Mapping`: Two types of mapping were performed: one using laser data and the other using the 3D lidar data converted to 2D format. The conversion of the 3D lidar data provided more accurate results compared to laser data. Four maps were generated using two different bag files, with the map obtained from the second bag file and 3D lidar data conversion considered the best among the four. Some noisy points were removed from the map using GIMP.
- `Navigation`: The navigation system utilized the best map obtained from the mapping process. AMCL (Adaptive Monte Carlo Localization) was used for localization, utilizing the known map and sensor model to correct odometry. The robot was navigated using the move_base package, with goals provided from a CSV file (`waypoints.csv`). The global planner utilized the Global Planner technique, while the local planner utilized the TEB (Timed Elastic Band) local planner. The robot's size, sensor parameters, and planner settings were appropriately configured. Inflation radius was set to allow the robot to navigate comfortably in the map while avoiding narrow spaces where it could get stuck.

## Running the Project
To run the project, follow the steps below:

1. Launch the mapping process by executing one of the following commands:
- For laser mapping: `roslaunch second_project mapping_laser.launch`
- For 3D lidar mapping: `roslaunch second_project mapping_lidar.launch`
Make sure to run the corresponding bag file in a separate terminal window.
2. Once the mapping process is complete, launch the navigation system by executing the following command:
`roslaunch second_project robot_amcl.launch`
This will start the robot simulation in Stage along with the navigation stack. RViz will also be launched to visualize the robot's movements and the map.

3. To provide the robot with a set of waypoints for autonomous navigation, make sure you have a CSV file named `waypoints.csv` in the project directory. 

4. Once the navigation system is launched and the `waypoints.csv` file is ready, the robot will start moving towards the waypoints autonomously. You can observe its progress in RViz and the terminal.

## Dependencies
The ROS project has the following dependencies:

- ROS (Robot Operating System): Make sure you have ROS installed on your system. The project was developed and tested on ROS Kinetic, but it should work with other versions as well.
- ROS packages: The project utilizes several ROS packages, including `slam_toolbox`, `move_base`, `amcl`, and `stage_ros`. Make sure these packages are installed in your ROS environment.
- Bag Files: The project requires input data in the form of bag files. You can either use the provided bag files or record your own data.

## Contributing
This project has been implemented with the contribution of Sara Zoccheddu.

## License
This project is licensed under the MIT License.


