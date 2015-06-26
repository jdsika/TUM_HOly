#include "walk.h"

#include "core.h"
#include "poses/poses.h"
#include "poses/parser.h"

#define DEBUG 1

Walk::Walk(Core *core) : core{core}
{

    init_StateMachine();

}

Walk::~Walk()
{

}

void Walk::executeStateMachine()
{
    // parse files before each walking attempt
    if(!Parser::parseRoboPositions(Parser::filename)) return;

    // all loaded walking poses will be executed
    // do not start wih position_default
    for(int i = 0; i < Parser::walkingPoses.size();++i)
    {
        if(!ros::ok()) return;
        core->setPoseTarget(Parser::walkingPoses.at(i)).move();
    }

    if(!ros::ok()) return;
    ros::spinOnce();
}

void Walk::StateMachine() {

    // update parameters
    walk_poses.set_step_height(0.01); // max 0.05
    walk_poses.set_turning_angle(core->get_turning_angle());
    walk_poses.set_step_length(core->getStep_length()); // max 0.033
    walk_poses.update();

    //**********************STAND***********************

    if (walk_fsm == Walk_FSM::STAND) {

        if (!core->get_stop()) {
            walk_fsm = Walk_FSM::INIT;
            core->set_isstanding(false);
            if (DEBUG) ROS_INFO("INIT");
        }
    }

    //**********************INIT***********************

    else if (walk_fsm == Walk_FSM::INIT) {

        //Init
        if (init_fsm==iSHIFT_LEFT) {
            if (core->get_goal_success()) {
                core->setPoseTarget(walk_poses.init_shift_toleft).move(core->get_vel_slow());
                init_fsm=iFWD_RIGHT;
            }
        }
        /*else if (init_fsm==iLIFT_RIGHT) {
            if (core->get_goal_success()) {
                core->setPoseTarget(poses.init_lift_right).move(core->get_vel());
                init_fsm=iFWD_RIGHT;
            }
        }*/
        else if (init_fsm==iFWD_RIGHT) {
            if (core->get_goal_success()) {
                core->setPoseTarget(walk_poses.init_fwd_right).move(core->get_vel());
                init_fsm=iDUAL_RIGHT;
            }
        }
        else if (init_fsm==iDUAL_RIGHT) {
            if (core->get_goal_success()) {
                core->setPoseTarget(walk_poses.init_dual_right).move(core->get_vel());
                init_fsm=iSHIFT_FRONT_RIGHT;
            }
        }
        else if (init_fsm==iSHIFT_FRONT_RIGHT) {
            if (core->get_goal_success()) {
                core->setPoseTarget(walk_poses.init_shift_frontright).move(core->get_vel_slow());
                init_fsm=iSHIFT_LEFT;
                // Go to Loop
                if (!core->get_stop()) {
                    walk_fsm = Walk_FSM::LOOP;
                    if (DEBUG) ROS_INFO("LOOP");
                }
                else {
                    walk_fsm = Walk_FSM::STOP;
                    if (DEBUG) ROS_INFO("STOP");
                }
            }
        }
    }

    //**********************LOOP***********************

    else if (walk_fsm == Walk_FSM::LOOP) {

        // Loop
       /* if (loop_fsm==lLIFT_LEFT) {
            if (core->get_goal_success()) {
                core->setPoseTarget(poses.loop_lift_left).move(core->get_vel());
                loop_fsm=lFWD_LEFT;
            }
        }*/
        if (loop_fsm==lFWD_LEFT) {
            if (core->get_goal_success()) {
                core->setPoseTarget(walk_poses.loop_fwd_left).move(core->get_vel());
                loop_fsm=lDUAL_LEFT;
            }
        }
        else if (loop_fsm==lDUAL_LEFT) {
            if (core->get_goal_success()) {
                core->setPoseTarget(walk_poses.loop_dual_left).move(core->get_vel());
                loop_fsm=lSHIFT_FRONT_LEFT;
            }
        }
        else if (loop_fsm==lSHIFT_FRONT_LEFT) {
            if (core->get_goal_success()) {
                core->setPoseTarget(walk_poses.loop_shift_frontleft).move(core->get_vel_slow());
                if (!core->get_stop()) {
                    loop_fsm=lFWD_RIGHT;
                }
                else {
                    loop_fsm=lFWD_LEFT;
                    walk_fsm = Walk_FSM::STOP;
                    stop_fsm=sFWD_RIGHT;
                    if (DEBUG) ROS_INFO("STOP");
                }

            }
        }
        /*else if (loop_fsm==lLIFT_RIGHT) {
            if (core->get_goal_success()) {
                core->setPoseTarget(poses.loop_lift_right).move(core->get_vel());
                loop_fsm=lFWD_RIGHT;
            }
        }*/
        else if (loop_fsm==lFWD_RIGHT) {
            if (core->get_goal_success()) {
                core->setPoseTarget(walk_poses.loop_fwd_right).move(core->get_vel());
                loop_fsm=lDUAL_RIGHT;
            }
        }
        else if (loop_fsm==lDUAL_RIGHT) {
            if (core->get_goal_success()) {
                core->setPoseTarget(walk_poses.loop_dual_right).move(core->get_vel());
                loop_fsm=lSHIFT_FRONT_RIGHT;
            }
        }
        else if (loop_fsm==lSHIFT_FRONT_RIGHT) {
            if (core->get_goal_success()) {
                core->setPoseTarget(walk_poses.loop_shift_frontright).move(core->get_vel_slow());
                // Go to stop if control input
                if (!core->get_stop()) {
                    loop_fsm=lFWD_LEFT;
                }
                else {
                    loop_fsm=lFWD_RIGHT;
                    walk_fsm = Walk_FSM::STOP;
                    stop_fsm=sFWD_LEFT;
                    if (DEBUG) ROS_INFO("STOP");
                }
            }
        }
    }

    //**********************STOP***********************

    else if (walk_fsm == Walk_FSM::STOP) {

        // Stop
        /*if (stop_fsm==sLIFT_LEFT) {
            if (core->get_goal_success()) {
                core->setPoseTarget(poses.stop_lift_left).move(core->get_vel());
                stop_fsm=sFWD_LEFT;
            }
        }*/
        if (stop_fsm==sFWD_LEFT) {
            if (core->get_goal_success()) {
                core->setPoseTarget(walk_poses.stop_fwd_left).move(core->get_vel());
                stop_fsm=sDEFAULT;
            }
        }
        else if (stop_fsm==sFWD_RIGHT) {
            if (core->get_goal_success()) {
                core->setPoseTarget(walk_poses.stop_fwd_right).move(core->get_vel());
                stop_fsm=sDEFAULT;
            }
        }
        else if (stop_fsm==sDEFAULT) {
            if (core->get_goal_success()) {
                core->setPoseTarget(walk_poses.pose_default).move(core->get_vel());
                //stop_fsm=sFWD_LEFT;
                // Go to Stand
                core->set_isstanding(true);
                if (DEBUG) ROS_INFO("STAND");
                walk_fsm = Walk_FSM::STAND;
            }
        }

    }
}

void Walk::init_StateMachine() {
    // Init to first element in FSM
    walk_fsm = Walk_FSM::STOP;
    init_fsm = iSHIFT_LEFT;
    loop_fsm = lFWD_LEFT;
    stop_fsm = sDEFAULT;
}
