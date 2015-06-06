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

/* L_EB  - Left Elbow
 * L_SFE - Left Biceps (bends)
 * L_SAA - Left Shoulder (rotates)
 * L_HAA - Left Hip (rotates)
 * L_HR  - Left Hip (bends)
 * L_HFE - Left Hip abductor (sideways)
 * L_KFE - Left Knee
 * L_AFE - Left Ankle (bends)
 * L_AR  - Left Foot (sideways rotate)
 * R_EB  - Right Elbow
 * R_SFE - Right Biceps (bends)
 * R_SAA - Right Shoulder (rotates)
 * R_HAA - Right Hip (rotates)
 * R_HR  - Right Hip (bends)
 * R_HFE - Right Hip abductor (sideways)
 * R_KFE - Right Knee
 * R_AFE - Right Ankle (bends)
 * R_AR  - Right Foot (sideways rotate)
 */



int main(int argc, char **argv)
{
    Core core(argc, argv);
    Walk walk(&core);

    Walk::pose ap = walk.getCurrentPose(Core::Limb::RIGHT_HAND);

    //ap.pitch += 45*M_PI/180.0;
    // ap.roll += 45*M_PI/180.0;
    //ap.yaw += 45*M_PI/180.0;
    //ap.y += 0.03;
    //ap.y += 3.00;
    ap.z += 0.03;

//    geometry_msgs::Pose p = walk.transformToPlan(Core::Limb::RIGHT_HAND, ap);

    core.setPoseTarget(Core::Limb::RIGHT_HAND, ap.toGeoPose());
    core.move();

    // running the program at a rate of 20Hz
    std::cout << "Init done" << std::endl;
    ros::Rate rate(20);
//    while (ros::ok()) {
//        // sleep as long as the 20Hz cycle frequency is met
//        rate.sleep();
//        // process callbacks in custom loops
//        ros::spinOnce();
//    }

    return 0;

}
