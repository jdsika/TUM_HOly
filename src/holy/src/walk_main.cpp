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
//    Poses::getWorkingDirectory();
//    Poses::printCSVRows(filename);
//    Poses::parseRoboPositions(Poses::filename);

//    std::cout << "Size walkingPoses: " << Poses::walkingPoses.size() << std::endl;

//    for (int i = 0; Poses::walkingPoses.size(); i++)
//        (Poses::walkingPoses[i]).printInfo();

//    return 0;

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

        //walk.executeStateMachine();

        core.setPoseTarget(Poses::pose_shift_weight_toright).move();
        core.setPoseTarget(Poses::pose_lift_left_foot).move();
        core.setPoseTarget(Poses::pose_left_foot_advance_forward).move();
        core.setPoseTarget(Poses::pose_left_foot_advanced_down).move();
        core.setPoseTarget(Poses::pose_left_foot_advanced_shiftweighttoleft).move();

        core.setPoseTarget(Poses::pose_shift_weight_toleft).move();
        core.setPoseTarget(Poses::pose_lift_right_foot).move();
        core.setPoseTarget(Poses::pose_right_foot_advance_forward).move();
        core.setPoseTarget(Poses::pose_right_foot_advanced_down).move();
        core.setPoseTarget(Poses::pose_right_foot_advanced_shiftweighttoright).move();

        ros::spinOnce();
        if(!ros::ok()) break;
//        rate.sleep();

    }

    return 0;

}
