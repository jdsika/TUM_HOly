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
#include "poses/parser.h"
#include "poses/robopose.h"


int main(int argc, char **argv)
{
    Parser::getWorkingDirectory();
    Core core(argc, argv);
    Walk walk(&core);

    ros::Rate rate(1);
    // In Start Position gehen
    core.setPoseTarget(Poses::pose_default).move();

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

        /*if (walk.walk_fsm==Walk::STAND) {
            // Stand

            if (!core.get_stop()) {
                walk.walk_fsm=Walk::INIT;
            }
        }

        else if (walk.walk_fsm==Walk::INIT) {

            //Init
            if (walk.init_fsm==Walk::iSHIFT_LEFT) {
                if (core.get_goal_success()) {
                    core.setPoseTarget(Poses::init_shift_toleft).move(core.get_vel());
                    walk.init_fsm=Walk::iLIFT_RIGHT;
                }
            }
            else if (walk.init_fsm==Walk::iLIFT_RIGHT) {
                if (core.get_goal_success()) {
                    core.setPoseTarget(Poses::init_lift_right).move(core.get_vel());
                    walk.init_fsm=Walk::iFWD_RIGHT;
                }
            }
            else if (walk.init_fsm==Walk::iFWD_RIGHT) {
                if (core.get_goal_success()) {
                    std::cout<<"ja";
                    core.setPoseTarget(Poses::init_fwd_right).move(core.get_vel());
                    walk.init_fsm=Walk::iDUAL_RIGHT;
                }
            }
            else if (walk.init_fsm==Walk::iDUAL_RIGHT) {
                if (core.get_goal_success()) {
                    core.setPoseTarget(Poses::init_dual_right).move(core.get_vel());
                    walk.init_fsm=Walk::iSHIFT_FRONT_RIGHT;
                }
            }
            else if (walk.init_fsm==Walk::iSHIFT_FRONT_RIGHT) {
                if (core.get_goal_success()) {
                    core.setPoseTarget(Poses::init_shift_frontright).move(core.get_vel());
                    walk.init_fsm=Walk::iSHIFT_LEFT;
                    // Go to Loop
                    //walk.walk_fsm=Walk::LOOP;
                }
            }
        }

        else if (walk.walk_fsm==Walk::LOOP) {

            // Loop
            core.setPoseTarget(Poses::loop_lift_left).move(core.get_vel());
            core.setPoseTarget(Poses::loop_fwd_left).move(core.get_vel());
            core.setPoseTarget(Poses::loop_dual_left).move(core.get_vel());
            core.setPoseTarget(Poses::loop_shift_frontleft).move(core.get_vel());
            core.setPoseTarget(Poses::loop_lift_right).move(core.get_vel());
            core.setPoseTarget(Poses::loop_fwd_right).move(core.get_vel());
            core.setPoseTarget(Poses::loop_dual_right).move(core.get_vel());
            core.setPoseTarget(Poses::loop_shift_frontright).move(core.get_vel());

            // Go to stop if control input
            if (core.get_stop()) {
                walk.walk_fsm=Walk::STOP;
            }
        }
        else if (walk.walk_fsm==Walk::STOP) {

            // Stop
            core.setPoseTarget(Poses::stop_lift_left).move(core.get_vel());
            core.setPoseTarget(Poses::stop_fwd_left).move(core.get_vel());
            core.setPoseTarget(Poses::pose_default).move(core.get_vel());
            walk.walk_fsm=Walk::STAND;
        }*/
        rate.sleep();
        ros::spinOnce();
    }
    //walk.executeStateMachine();*/

    return 0;

}
