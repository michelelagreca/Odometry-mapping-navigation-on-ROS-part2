For what concern the TF and the various transformations, in the tf_publisher a transformation between the odom node 
(that correspond to the world) and the t265 frame (that correspond to the robot) has been made, in order to convert the odometry topic into a tf.

The structure of the tree is similar to the one of the bag file we were provided: it is present a dynamic transformation between odom and t265, 
and static tf form t265 to base_footprint, from it to base_link, and from base_link to the velodyne and laser sensors. 
The only transformations that miss are related to base_link and imu_link, lidar_link, body_link and oak-d-base-frame, that were probably out of scope for the project.
The static tf have also some specific values taken from the tf of the bag data.

After having setted the tree, mapping has been implemented using slam toolbox, using laser and the converison in 2d of the LIDAR data.
The conversion has been done using basically the same value as we have seen during the lecture, and the result was quite good:
the map obtained with the laser data is less accurate wrt the one obtained using the LIDAR. And the conversion of the LIDAR seems to be more accurate than the lasers.

The configurations of the parameters in the yaml conf file of the mapping has been keeped the same as the one of the lecture.

To run the mapping, it is necessary to run the launch file related of the laser mapping, or the launch file related to the LIDAR mapping:
    roslaunch second_project mapping_laser.launch
    roslaunch second_project mapping_lidar.launch
and in another window run a bag file.

4 maps has been built, 2 with the first bag file, and 2 with the second one. For each bag, one map using lasers and one using the 2d conversion of the LIDAR. 
With the second bag a slighly better map has been obtained.
In map_row folder there are the 4 row maps. The map obtained using the second bag file and LIDAR data will be used for the navigation,
because it seems the best among the 4 maps. Moreover, few noise points has been removed using GIMP.

For the navigation, we have used the best map found in the mapping. 
Amcl is used, that is performing localization using the known map, using particle filtering localization (monte carlo localization)
and so correcting the odometry using a sensor model.
To allow the robot navigating, a .csv file is used to tell the robot where to go, and the navigation is implemented using move_base.
The move_base node links together a global and local planner to accomplish its global navigation task through the desired positions (4 positions are 
present in the .csv file). the sensor used is a laser.

First of all it is necessary to set the parameters of the robot (in robot.inc file).
It has the size specified in the slide (0.6 x 0.4) and the type is differential, since it is a good approximation of the original
type of the robot (skid steering).
We assume no error in the odometry. 
The velocity and accelleration of the robot is also quite standard.
The sensor parameters are quite similar to the one of the bag file: the fov is 180°, similar to the laser of the bag file.

For the map, it is fixed in the origin of both stage and rviz, and moreover the map size in both environment is the same.
The robot is initially positioned in a simple position that is in an empty space. And the robot footprint is spread around 
this point so that the baricenter is in the center (for both stage and rviz)

For the global planner (that will create the full trajectory) Global Planner technique has been used. 
navfn uses Dijkstra's algorithm, while global_planner is computed with more flexible options. Carrot is the simplest globalplanner so 
it may not be enough for this task.
In general, the infaltion radius is 0.2, and it is a good value for this type of robot and map, because it allow 
the robot to navigate easly in the map but also tu avoid going to very narrow places where it can stuck.

For the local planning, teb has been used, because it is typically better than dwa. It check the area surrounding the robot for obstacle and it can adjust the
global path accordingly. The inflation radius is still 0.2.
It is computed more fequently than the global planning.

The paramters of the amcl.launch file are the default ones, except the initial position of the robot.

Finally, for the navigation using point in csv file, the action mechanism has been used.
The navigation.cpp contain a node that is a simple action client, that send a goal to the move_base node (a server action).
Then move_base move the robot, and when the goal is reached (in reasonable time), the result is sent to the client, and it send the next goal.

To run the navigation part:
    roslaunch second_project robot_amcl.launch




