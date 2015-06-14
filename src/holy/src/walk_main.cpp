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

static const std::string filename = "positions.csv";


int main(int argc, char **argv)
{
    //Poses::getWorkingDirectory();
    //Poses::printCSVRows(filename);
    Poses::parseRoboPositions(filename);

    std::cout << "Size walkingPoses: " << Poses::walkingPoses.size() << std::endl;

    for (int i = 0; Poses::walkingPoses.size(); i++)
        (Poses::walkingPoses[i]).printInfo();

    //return 0;

    Core core(argc, argv);
    Walk walk(&core);

    ros::Duration(2.0).sleep();

    // In Start Position gehen
    core.setPoseTarget(Poses::pose_default).move();


    ros::Duration(2.0).sleep();

    ros::Rate rate(0.25);

    while(true) {

	core.setPoseTarget(Poses::pose_default).move();
   	core.setPoseTarget(Poses::pose_shift_weight_toright).move();
        core.setPoseTarget(Poses::pose_lift_left_foot).move();
 	core.setPoseTarget(Poses::pose_left_foot_forward).move();
        core.setPoseTarget(Poses::pose_shift_weight_toleft).move();
 	core.setPoseTarget(Poses::pose_lift_right_foot).move();
	core.setPoseTarget(Poses::pose_left_foot_forward).move();
	core.setPoseTarget(Poses::pose_shift_weight_toright).move();
        core.setPoseTarget(Poses::pose_default).move();

        ros::spinOnce();
        if(!ros::ok()) break;
        rate.sleep();

    }

    return 0;

}
