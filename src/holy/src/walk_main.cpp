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


    Poses::pose_default.printInfo();
    (Poses::pose_default
            + RoboPose(std::vector<LimbPose> {
                           LimbPose(Core::Limb::RIGHT_FOOT, d2r(0), d2r(90), d2r(0), 0.06, 0, 0),
                           LimbPose(Core::Limb::LEFT_HAND, d2r(0), d2r(90), d2r(0), -0.06, 0, 0),
                       })
            ).printInfo();

//    return 0;

    // In Start Position gehen
    core.setPoseTarget(Poses::pose_default).move();

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
