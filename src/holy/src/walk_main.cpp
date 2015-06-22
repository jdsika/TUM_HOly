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

#define DEBUG 1

int main(int argc, char **argv)
{
    ros::init(argc, argv, "holy_walk");
    Parser::getWorkingDirectory();
    Core core(argc, argv);
    Walk walk(&core);
    Poses poses;

    //ros::Subscriber goal_sub;
    //goal_sub = n.subscribe<actionlib_msgs::GoalStatusArray>("/move_group/status", 10, &Core::goalCallback, &core);
    ros::Rate rate(100);
    // In Start Position gehen
    core.setPoseTarget(poses.pose_default).move();

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
    poses.set_step_height(0.01); // max 0.05
    while(ros::ok()) {

        // update parameters
        poses.set_turning_angle(core.get_turning_angle());
        poses.set_step_length(core.getStep_length()); // max 0.033
        poses.update();
        //**********************STAND***********************

        if (walk.walk_fsm==Walk::STAND) {

            if (!core.get_stop()) {
                walk.walk_fsm=Walk::INIT;
                if (DEBUG) ROS_INFO("INIT");
            }
        }

        //**********************INIT***********************

        else if (walk.walk_fsm==Walk::INIT) {

            //Init
            if (walk.init_fsm==Walk::iSHIFT_LEFT) {
                if (core.get_goal_success()) {
                    core.setPoseTarget(poses.init_shift_toleft).move(core.get_vel()/3);
                    walk.init_fsm=Walk::iFWD_RIGHT;
                }
            }
            /*else if (walk.init_fsm==Walk::iLIFT_RIGHT) {
                if (core.get_goal_success()) {
                    core.setPoseTarget(poses.init_lift_right).move(core.get_vel());
                    walk.init_fsm=Walk::iFWD_RIGHT;
                }
            }*/
            else if (walk.init_fsm==Walk::iFWD_RIGHT) {
                if (core.get_goal_success()) {
                    core.setPoseTarget(poses.init_fwd_right).move(core.get_vel());
                    walk.init_fsm=Walk::iDUAL_RIGHT;
                }
            }
            else if (walk.init_fsm==Walk::iDUAL_RIGHT) {
                if (core.get_goal_success()) {
                    core.setPoseTarget(poses.init_dual_right).move(core.get_vel());
                    walk.init_fsm=Walk::iSHIFT_FRONT_RIGHT;
                }
            }
            else if (walk.init_fsm==Walk::iSHIFT_FRONT_RIGHT) {
                if (core.get_goal_success()) {
                    core.setPoseTarget(poses.init_shift_frontright).move(core.get_vel()/3);
                    walk.init_fsm=Walk::iSHIFT_LEFT;
                    // Go to Loop
                    if (!core.get_stop()) {
                        walk.walk_fsm=Walk::LOOP;
                        if (DEBUG) ROS_INFO("LOOP");
                    }
                    else {
                        walk.walk_fsm=Walk::STOP;
                        if (DEBUG) ROS_INFO("STOP");
                    }


                }
            }
        }

        //**********************LOOP***********************

        else if (walk.walk_fsm==Walk::LOOP) {

            // Loop
           /* if (walk.loop_fsm==Walk::lLIFT_LEFT) {
                if (core.get_goal_success()) {
                    core.setPoseTarget(poses.loop_lift_left).move(core.get_vel());
                    walk.loop_fsm=Walk::lFWD_LEFT;
                }
            }*/
            if (walk.loop_fsm==Walk::lFWD_LEFT) {
                if (core.get_goal_success()) {
                    core.setPoseTarget(poses.loop_fwd_left).move(core.get_vel());
                    walk.loop_fsm=Walk::lDUAL_LEFT;
                }
            }
            else if (walk.loop_fsm==Walk::lDUAL_LEFT) {
                if (core.get_goal_success()) {
                    core.setPoseTarget(poses.loop_dual_left).move(core.get_vel());
                    walk.loop_fsm=Walk::lSHIFT_FRONT_LEFT;
                }
            }
            else if (walk.loop_fsm==Walk::lSHIFT_FRONT_LEFT) {
                if (core.get_goal_success()) {
                    core.setPoseTarget(poses.loop_shift_frontleft).move(core.get_vel()/3);
                    if (!core.get_stop()) {
                        walk.loop_fsm=Walk::lFWD_RIGHT;
                    }
                    else {
                        walk.loop_fsm=Walk::lFWD_LEFT;
                        walk.walk_fsm=Walk::STOP;
                        walk.stop_fsm=Walk::sFWD_RIGHT;
                        if (DEBUG) ROS_INFO("STOP");
                    }

                }
            }
            /*else if (walk.loop_fsm==Walk::lLIFT_RIGHT) {
                if (core.get_goal_success()) {
                    core.setPoseTarget(poses.loop_lift_right).move(core.get_vel());
                    walk.loop_fsm=Walk::lFWD_RIGHT;
                }
            }*/
            else if (walk.loop_fsm==Walk::lFWD_RIGHT) {
                if (core.get_goal_success()) {
                    core.setPoseTarget(poses.loop_fwd_right).move(core.get_vel());
                    walk.loop_fsm=Walk::lDUAL_RIGHT;
                }
            }
            else if (walk.loop_fsm==Walk::lDUAL_RIGHT) {
                if (core.get_goal_success()) {
                    core.setPoseTarget(poses.loop_dual_right).move(core.get_vel());
                    walk.loop_fsm=Walk::lSHIFT_FRONT_RIGHT;
                }
            }
            else if (walk.loop_fsm==Walk::lSHIFT_FRONT_RIGHT) {
                if (core.get_goal_success()) {
                    core.setPoseTarget(poses.loop_shift_frontright).move(core.get_vel()/3);
                    walk.loop_fsm=Walk::lFWD_LEFT;
                    // Go to stop if control input
                    if (core.get_stop()) {
                        walk.walk_fsm=Walk::STOP;
                        if (DEBUG) ROS_INFO("STOP");
                    }
                }
            }
        }

        //**********************STOP***********************

        else if (walk.walk_fsm==Walk::STOP) {

            // Stop
            /*if (walk.stop_fsm==Walk::sLIFT_LEFT) {
                if (core.get_goal_success()) {
                    core.setPoseTarget(poses.stop_lift_left).move(core.get_vel());
                    walk.stop_fsm=Walk::sFWD_LEFT;
                }
            }*/
            if (walk.stop_fsm==Walk::sFWD_LEFT) {
                if (core.get_goal_success()) {
                    core.setPoseTarget(poses.stop_fwd_left).move(core.get_vel());
                    walk.stop_fsm=Walk::sDEFAULT;
                }
            }
            else if (walk.stop_fsm==Walk::sFWD_RIGHT) {
                if (core.get_goal_success()) {
                    core.setPoseTarget(poses.stop_fwd_right).move(core.get_vel());
                    walk.stop_fsm=Walk::sDEFAULT;
                }
            }
            else if (walk.stop_fsm==Walk::sDEFAULT) {
                if (core.get_goal_success()) {;
                    core.setPoseTarget(poses.pose_default).move(core.get_vel());
                    walk.stop_fsm=Walk::sFWD_LEFT;
                    // Go to Stand
                    walk.walk_fsm=Walk::STAND;
                    if (DEBUG) ROS_INFO("STAND");
                }
            }

        }
        rate.sleep();
        ros::spinOnce();
    }
    //walk.executeStateMachine();*/

    return 0;

}
