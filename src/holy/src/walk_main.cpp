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
#include "poses/parser.h"
#include "poses/robopose.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "holy_walk");
    Parser::getWorkingDirectory();
    Core core(argc, argv);
    Walk walk(&core);
    Poses default_poses;
    //ros::Subscriber goal_sub;
    //goal_sub = n.subscribe<actionlib_msgs::GoalStatusArray>("/move_group/status", 10, &Core::goalCallback, &core);
    ros::Rate rate(100);
    // In Start Position gehen
    core.setPoseTarget(default_poses.pose_default).move();

    ros::Duration(0.5).sleep();

    //
    // LimbPose parameterization test
    //

    /*LimbPose lp = Poses::pose_default.getLimb(Core::Limb::RIGHT_HAND);

    // fuege neuen parameter Roll-Influence hinzu, der zum standard roll-wert 10* den input wert hinzufuegt
    lp.setParameterAdd("Roll-Influence", 10, 0, 0 , 0, 0, 0);

    // setze den input fuer Roll-Influence auf 2. Roll sollte jetzt standard * 1.0 + 10*2 sein.
    lp.setParameterInput("Roll-Influence", 2.0);

    core.setPoseTarget(lp).move();
    ros::Duration(2.0).sleep();


    // Parameter "entfernen" indem einfluss auf 0 gesetzt wird
    lp.setParameterAdd("Roll-Influence", 0, 0, 0 , 0, 0, 0);

    // Neuen Parameter hinzufuegen der roll wert auf standard * 1.3*input aendert
    lp.setParameterMul("Roll-Influence Multiplikativ", 1.3, 0, 0, 0, 0, 0);

    // input fuer parameter
    lp.setParameterInput("Roll-Influence Multiplikativ", -3.0);

    // Roll sollte standard * 1.3*(-2.0) + 0 sein
    core.setPoseTarget(lp).move();

    return 0;*/


    //std::cout << "Walk (w) or Stairs (s) ?" << std::endl;
    //std::coud << "Your choice: ";
    //std::cin >> input;

    while(ros::ok()) {

        walk.StateMachine();
        rate.sleep();
        ros::spinOnce();
    }
    //walk.executeStateMachine();*/

    return 0;

}
