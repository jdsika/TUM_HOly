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

    // In Start Position gehen
    Walk::pose defPosLF = walk.getCurrentPose(Core::Limb::LEFT_FOOT);
    Walk::pose defPosRF = walk.getCurrentPose(Core::Limb::RIGHT_FOOT);
    std::cout << defPosRF.x << std::endl;
    defPosLF.z += 0.02;
    defPosRF.z += 0.02;
    std::cout << defPosRF.x << std::endl;
    Walk::pose ap;

    ros::Rate rate(0.25);

    while(true) {

	core.setPoseTarget(Core::Limb::LEFT_FOOT, defPosLF.toGeoPose());
	core.setPoseTarget(Core::Limb::RIGHT_FOOT, defPosRF.toGeoPose());
	core.move();
	std::cout << "Default Pose done"<<std::endl;
	Walk::pose defPosRF = walk.getCurrentPose(Core::Limb::RIGHT_FOOT);
	std::cout << defPosRF.x << std::endl;
	rate.sleep();
        ros::spinOnce();
        // Gewicht nach Links
  	if(!ros::ok()) break;
        ap = defPosRF;
        ap.x += 0.06;
        ap.pitch += 8*M_PI/180.0;
        core.setPoseTarget(Core::Limb::RIGHT_FOOT, ap.toGeoPose());

        ap = defPosLF;
        ap.pitch += 8*M_PI/180.0;
        core.setPoseTarget(Core::Limb::LEFT_FOOT, ap.toGeoPose());

        core.move();

        rate.sleep();
        ros::spinOnce();

        // Rechten Fuß anheben
        if(!ros::ok()) break;
        ap = walk.getCurrentPose(Core::Limb::RIGHT_FOOT);
        ap.z += 0.05;
        ap.pitch += 4*M_PI/180.0;
        core.setPoseTarget(Core::Limb::RIGHT_FOOT, ap.toGeoPose());

        ap = walk.getCurrentPose(Core::Limb::LEFT_FOOT);
        ap.x -= -0.025;
        ap.y -= -0.008;
        ap.pitch += 4*M_PI/180.0;
        core.setPoseTarget(Core::Limb::LEFT_FOOT, ap.toGeoPose());

        core.move();

        rate.sleep();
        ros::spinOnce();




    }

    return 0;

}
