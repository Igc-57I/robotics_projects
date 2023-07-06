#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <iostream>
#include <array>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <sstream>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
    ros::init(argc, argv, "simple_navigation_goals");
    ros::NodeHandle n;

    // read file path from parameter file
    std::string path;
    if (argc == 2) {
        path = argv[1];
    } else {
        ROS_INFO("Error: no path specified");
        return 1;
    }

    // read waypoints.csv file whith c++ fstream and for each line copy the fist value in x array, the second in y array and the third in heading array
    std::ifstream file(path, std::ifstream::in);

    if (!file.is_open()) {
        ROS_INFO("Error opening file");
        return 1;
    }
    ROS_INFO("Reading waypoints.csv file");
    std::string line;
    std::vector<std::vector<double>> values;
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::vector<double> row;
        double value;
        while (ss >> value) {
            row.push_back(value);
            if (ss.peek() == ',') {
                ss.ignore();
            }
        }
        values.push_back(row);
    }

    ROS_INFO("Copying values in x, y and heading array");
    // create 3 array x, y and heading and copy the values from the vector values, the first value of each row in x array, the second in y array and the third in heading array
    double x [values.size()];
    double y [values.size()];
    double heading [values.size()];
    for (int i = 0; i < values.size(); i++) {
        x[i] = values[i][0];
        y[i] = values[i][1];
        heading[i] = values[i][2];
    }

    int lenght0f_Array = sizeof(x) / sizeof(x[0]);

    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;

    //we'll send a goal to the robot to move 1 meter forward
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    for( int i = 0; i < lenght0f_Array; i = i + 1 ) {
        goal.target_pose.pose.position.x = x[i];
        goal.target_pose.pose.position.y = y[i];
        goal.target_pose.pose.orientation.z = heading[i];
        goal.target_pose.pose.orientation.w = 1.0;
        ROS_INFO("Sending goal %d", i);
        ac.sendGoal(goal);

        ac.waitForResult();

        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("Goal reached");
        else
            ROS_INFO("The robot failed to reach the goal for some reason");
    }

    return 0;
}