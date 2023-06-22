#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

std::vector<std::vector<double>> readWaypointsFromCSV(const std::string& file_path)
{
    std::vector<std::vector<double>> waypoints;

    std::ifstream file(file_path);
    if (!file.is_open())
    {
        ROS_ERROR_STREAM("Failed to open CSV file: " << file_path);
        return waypoints;
    }

    std::string line;
    while (std::getline(file, line))
    {
        std::stringstream ss(line);
        std::vector<double> waypoint;
        std::string value;

        while (std::getline(ss, value, ','))
        {
            try
            {
                double val = std::stod(value);
                waypoint.push_back(val);
            }
            catch (const std::exception& e)
            {
                ROS_WARN_STREAM("Invalid value found in CSV file: " << value);
            }
        }

        if (!waypoint.empty())
        {
            waypoints.push_back(waypoint);
        }
    }

    file.close();
    return waypoints;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "navigation");
    ros::NodeHandle nh;

    std::string csv_file_path;

    std::vector<std::vector<double>> waypoints = readWaypointsFromCSV(csv_file_path);
    

    MoveBaseClient ac("move_base", true);
    while (!ac.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting...");
    }

    for (const auto& waypoint : waypoints)
    {
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.position.x = waypoint[0];
        goal.target_pose.pose.position.y = waypoint[1];
        goal.target_pose.pose.orientation.z = waypoint[2];
        goal.target_pose.pose.orientation.w = waypoint[3];

        ROS_INFO("Sending goal: x=%.2f, y=%.2f, theta=%.2f",
                 goal.target_pose.pose.position.x,
                 goal.target_pose.pose.position.y,
                 goal.target_pose.pose.orientation.z);

        ac.sendGoal(goal);

        ac.waitForResult();

        if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO("Goal reached!");
        }
        else
        {
            ROS_WARN("Failed to reach the goal.");
        }
    }

    return 0;
}
