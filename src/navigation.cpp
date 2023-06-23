#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <fstream>
#include <iostream>
#include <vector>
#include <string>
#include <tf2/LinearMath/Quaternion.h>


int main (int argc, char **argv)
{ 
	ros::init(argc, argv, "navigation");
	std::ifstream file("/home/michelelagreca/Documents/Robotics/ROS/src/second_project/waypoints.csv");
    if (!file) {
        std::cout << "Error opening file!" << std::endl;
        return 1;
    }

    std::vector<std::array<float, 3>> data;
    std::string line;
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::array<float, 3> values;
        char delimiter;
        if (!(iss >> values[0] >> delimiter >> values[1] >> delimiter >> values[2])) {
            std::cout << "Error: Invalid line format!" << std::endl;
            return 1;
        }
        if (delimiter != ',') {
            std::cout << "Error: Invalid delimiter!" << std::endl;
            return 1;
        }
        data.push_back(values);
    }

    // Print the contents of the array
    for (const auto& item : data) {
        for (const auto& value : item) {
            std::cout << value << " ";
        }
        std::cout << std::endl;
    }

    ROS_INFO("SIMPLE ACTION CLIENT ------> Waiting for the move_base action server to come up"); 
    
    
    
    // create the action client
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);
    
    //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("SIMPLE ACTION CLIENT ------> Waiting for the move_base action server to come up");
    }

    ROS_INFO("SIMPLE ACTION CLIENT ------> Action server started");

    int seq_num = 0;

    for (const auto& item : data) {
        
        float goal_x = item[0];
        float goal_y = item[1];
        float goal_heading = item[2];

        tf2::Quaternion q;
        q.setRPY( 0, 0, goal_heading);

        ROS_INFO("SIMPLE ACTION CLIENT ------> Sending a new goal: x = %f, y = %f, heading = %f rad,\n\t\tOrientation x = %f, Orientation y = %f, Orientation z = %f, Orientation w = %f", goal_x, goal_y, goal_heading, q.getX(), q.getY(), q.getZ(), q.getW());

        // send a goal to the action action
        move_base_msgs::MoveBaseGoal goal;

        goal.target_pose.header.seq = seq_num;
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.pose.position.x = goal_x;
        goal.target_pose.pose.position.y = goal_y;
        goal.target_pose.pose.position.z = 0;
        goal.target_pose.pose.orientation.x = q.getX();
        goal.target_pose.pose.orientation.y = q.getY();
        goal.target_pose.pose.orientation.z = q.getZ();
        goal.target_pose.pose.orientation.w = q.getW();

        ac.sendGoal(goal);
        
        //wait for the action to return
        bool finished_before_timeout = ac.waitForResult(ros::Duration(120)); //after 120 seconds if the goal is not reached we delete and we go ahead


        if (finished_before_timeout) // so computaton was done
        {
            actionlib::SimpleClientGoalState state = ac.getState();
            ROS_INFO("SIMPLE ACTION CLIENT ------> Action finished: %s",state.toString().c_str());
        }
        else
        {
            ROS_INFO("SIMPLE ACTION CLIENT ------> Action did not finish before the time out. Current goal is cancelled.");
            ac.cancelGoal ();
        }
        
        seq_num = seq_num + 1;
    }

    ROS_INFO("SIMPLE ACTION CLIENT ------> Navigation finished. All the goal has been processed.");


    return 0;
}
