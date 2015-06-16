#include <cmath>

#include <vector>
#include <string>
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <moveit/move_group_interface/move_group.h>
#include <geometry_msgs/Pose.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <tf/transform_listener.h>

#include "core.h"
#include "walk.h"
#include "poses/poses.h"
#include "poses/robopose.h"


int main(int argc, char **argv)
{
    Poses::getWorkingDirectory();
    Core core(argc, argv);
    Walk walk(&core);

    ros::Duration(2.0).sleep();

    // In Start Position gehen
    core.setPoseTarget(Poses::pose_default).move();

    ros::Duration(2.0).sleep();

    ros::Rate rate(0.25);

    //char input = '';

    //std::cout << "Walk (w) or Stairs (s) ?" << std::endl;
    //std::coud << "Your choice: ";
    //std::cin >> input;

    while(true) {

        walk.executeStateMachine();

        if(!ros::ok()) break;

    }

    return 0;

}
