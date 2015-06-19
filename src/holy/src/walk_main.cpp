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

    //ros::Duration(2.0).sleep();

    // In Start Position gehen
    core.setPoseTarget(Poses::pose_default).move();

    ros::Duration(0.5).sleep();

    ros::Rate rate(0.25);

    //char input = '';

    //std::cout << "Walk (w) or Stairs (s) ?" << std::endl;
    //std::coud << "Your choice: ";
    //std::cin >> input;
    while(ros::ok()) {
        // Init
        core.setPoseTarget(Poses::init_shift_toleft).move();
        core.setPoseTarget(Poses::init_lift_right).move();
        core.setPoseTarget(Poses::init_fwd_right).move();
        core.setPoseTarget(Poses::init_dual_right).move();
        core.setPoseTarget(Poses::init_shift_frontright).move();
        // Loop
        core.setPoseTarget(Poses::loop_lift_left).move();
        core.setPoseTarget(Poses::loop_fwd_left).move();
        core.setPoseTarget(Poses::loop_dual_left).move();
        core.setPoseTarget(Poses::loop_shift_frontleft).move();
        core.setPoseTarget(Poses::loop_lift_right).move();
        core.setPoseTarget(Poses::loop_fwd_right).move();
        core.setPoseTarget(Poses::loop_dual_right).move();
        core.setPoseTarget(Poses::loop_shift_frontright).move();
        // Stop
        core.setPoseTarget(Poses::stop_lift_left).move();
        core.setPoseTarget(Poses::stop_fwd_left).move();
        core.setPoseTarget(Poses::pose_default).move();
        //walk.executeStateMachine();

    }

    return 0;

}
