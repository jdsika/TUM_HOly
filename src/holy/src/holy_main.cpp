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

#include "holy_main.h"
#include "core.h"
#include "walk.h"
#include "fight.h"
#include "kinect.h"
#include "stairs.h"
#include "poses/parser.h"
#include "poses/robopose.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "holy_walk");
    Parser::getWorkingDirectory();
    Core core(argc, argv);
    Walk walk(&core);
    Stairs stairs(&core);
    Kinect kinect(&core);
    Fight fight(&core);

    HOLY_FSM holy_fsm = HOLY_FSM::WAIT;

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

    while(ros::ok()) {
        // STAND
        if (core.get_isstanding()==true) {
            if (core.get_buttons()[0]==1) {
                // GOTO STAIRS
                holy_fsm = HOLY_FSM::STAIRS;
            }
            else if (core.get_buttons()[1]==1) {
                // GOTO WALK
                holy_fsm = HOLY_FSM::WALK;
            }
            else if (core.get_buttons()[2]==1) {
                // GOTO KINECT
                holy_fsm = HOLY_FSM::KINECT;
            }
            else if (core.get_buttons()[3]==1) {
                // GOTO FIGHT
                holy_fsm = HOLY_FSM::FIGHT;
            }
        }
        // WALK
        if (holy_fsm == HOLY_FSM::WALK) {
            walk.StateMachine();
        }
        // STAIRS
        else if (holy_fsm == HOLY_FSM::STAIRS) {
            stairs.StateMachine();
        }
        // KINECT
        else if (holy_fsm == HOLY_FSM::KINECT) {
            kinect.StateMachine();
        }
        // FIGHT
        else if (holy_fsm == HOLY_FSM::FIGHT) {
            fight.StateMachine();
        }
        else {
            ROS_INFO("Waiting for action");
        }

        rate.sleep();
        ros::spinOnce();
    }
    //walk.executeStateMachine();*/

    return 0;
}
