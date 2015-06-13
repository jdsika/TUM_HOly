#include <cmath>

#include <vector>
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <moveit/move_group_interface/move_group.h>
#include <geometry_msgs/Pose.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <tf/transform_listener.h>

#include "core.h"
#include "walk.h"
#include "poses.h"


int main(int argc, char **argv)
{
    Core core(argc, argv);
    Walk walk(&core);

    ros::Duration(2.0).sleep();

    // In Start Position gehen
    core.setPoseTarget(Poses::pose_default).move();

    ros::Duration(2.0).sleep();

    return 0;

    ros::Rate rate(0.25);

    while(true) {

        core.setPoseTarget(Poses::pose_shift_weight_toleft).move();
        core.setPoseTarget(Poses::pose_lift_right_foot).move();
        core.setPoseTarget(Poses::pose_shift_weight_toleft).move();
        core.setPoseTarget(Poses::pose_default).move();

        ros::spinOnce();
        if(!ros::ok()) break;
        rate.sleep();

    }

    return 0;

}
