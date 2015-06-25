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

#include "holy_fsm_defines.h"
#include "core.h"
#include "walk.h"
#include "fight.h"
#include "kinect.h"
#include "stairs.h"
#include "poses/parser.h"
#include "poses/robopose.h"
#include "poses/poses.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "holy_walk");
    Parser::getWorkingDirectory();
    Core core(argc, argv);
    Walk walk(&core);
    Stairs stairs(&core);
    Kinect kinect(&core);
    Fight fight(&core);

    Holy_FSM holy_fsm = Holy_FSM::START;

    ros::Rate rate(100);

    /*//
    // LimbPose parameterization test
    //

    LimbPose lp = Poses::pose_default.getLimb(Core::Limb::RIGHT_HAND);

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

    return 0; */

    ROS_INFO("Waiting for action");

    while(ros::ok()) {
        // STAND
        if (core.get_isstanding()==true) {
            if (core.get_buttons()[10]==1
                    && holy_fsm != Holy_FSM::STAIRS) {
                // GOTO STAIRS
                holy_fsm = Holy_FSM::STAIRS;
                ROS_INFO("Holy_FSM -> STAIRS");
            }
            else if (core.get_buttons()[11]==1
                     && holy_fsm != Holy_FSM::WALK) {
                // GOTO WALK
                holy_fsm = Holy_FSM::WALK;
                ROS_INFO("Holy_FSM -> WALK");
            }
            else if (core.get_buttons()[8]==1
                     && holy_fsm != Holy_FSM::KINECT) {
                // GOTO KINECT
                holy_fsm = Holy_FSM::KINECT;
                ROS_INFO("Holy_FSM -> KINECT");
            }
            else if (core.get_buttons()[9]==1
                     && holy_fsm != Holy_FSM::FIGHT) {
                // GOTO FIGHT
                holy_fsm = Holy_FSM::FIGHT;
                ROS_INFO("Holy_FSM -> FIGHT");
            }
        }
        // WALK
        if (holy_fsm == Holy_FSM::WALK) {
            walk.StateMachine();
        }
        // STAIRS
        else if (holy_fsm == Holy_FSM::STAIRS) {
            stairs.StateMachine();
        }
        // KINECT
        else if (holy_fsm == Holy_FSM::KINECT) {
            kinect.StateMachine();
        }
        // FIGHT
        else if (holy_fsm == Holy_FSM::FIGHT) {
            fight.StateMachine();
        }
        else if(holy_fsm == Holy_FSM::START) {
            if(core.get_goal_success()) {
                // default position
                core.setPoseTarget(Poses::pose_default).move(core.get_vel());
                core.set_isstanding(true);
                ROS_INFO("Went to default position");
                holy_fsm = Holy_FSM::WAIT;
            }
        }
        else {
            // idle
        }

        rate.sleep();
        ros::spinOnce();
    }
    //walk.executeStateMachine();*/

    return 0;
}
