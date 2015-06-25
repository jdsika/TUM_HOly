#include "stairs.h"

#include "core.h"
#include "poses/poses.h"
#include "poses/parser.h"

#define DEBUG 1

Stairs::Stairs(Core *core) : core{core}
{

    init_StateMachine();

}

Stairs::~Stairs()
{

}

void Stairs::StateMachine() {

    // update parameters
    stairs_poses.set_step_height(0.05); // max 0.05
    stairs_poses.set_step_length(0.095); // max 0.033
    stairs_poses.update();

    //**********************STAND***********************

    if (stairs_fsm == Stairs_FSM::STAND) {

        if (!core->get_stop()) {
            stairs_fsm = Stairs_FSM::INIT;
            core->set_isstanding(false);
            if (DEBUG) ROS_INFO("INIT");
        }
    }

    //**********************INIT***********************

    else if (stairs_fsm == Stairs_FSM::INIT) {

        //Init
        if (init_fsm==iSHIFT_LEFT) {
            if (core->get_goal_success()) {
                core->setPoseTarget(stairs_poses.init_shift_toleft).move(core->get_vel()/3);
                init_fsm=iLIFT_RIGHT;
            }
        }
        else if (init_fsm==iLIFT_RIGHT) {
            if (core->get_goal_success()) {
                core->setPoseTarget(stairs_poses.init_lift_right).move(core->get_vel());
                init_fsm=iFWD_RIGHT;
            }
        }
        else if (init_fsm==iFWD_RIGHT) {
            if (core->get_goal_success()) {
                core->setPoseTarget(stairs_poses.init_fwd_right).move(core->get_vel());
                init_fsm=iDUAL_RIGHT;
            }
        }
        else if (init_fsm==iDUAL_RIGHT) {
            if (core->get_goal_success()) {
                core->setPoseTarget(stairs_poses.init_dual_right).move(core->get_vel());
                init_fsm=iSHIFT_FRONT_RIGHT;
            }
        }
        else if (init_fsm==iSHIFT_FRONT_RIGHT) {
            if (core->get_goal_success()) {
                core->setPoseTarget(stairs_poses.init_shift_frontright).move(core->get_vel()/3);
                init_fsm=iSHIFT_LEFT;
                // Go to Loop
                if (!core->get_stop()) {
                    stairs_fsm = Stairs_FSM::LOOP;
                    if (DEBUG) ROS_INFO("LOOP");
                }
                else {
                    stairs_fsm = Stairs_FSM::STOP;
                    if (DEBUG) ROS_INFO("STOP");
                }
            }
        }
    }

    //**********************LOOP***********************

    else if (stairs_fsm == Stairs_FSM::LOOP) {

        // Loop
        if (loop_fsm==lLIFT_LEFT) {
            if (core->get_goal_success()) {
                core->setPoseTarget(stairs_poses.loop_lift_left).move(core->get_vel());
                loop_fsm=lFWD_LEFT;
            }
        }
        if (loop_fsm==lFWD_LEFT) {
            if (core->get_goal_success()) {
                core->setPoseTarget(stairs_poses.loop_fwd_left).move(core->get_vel());
                loop_fsm=lDUAL_LEFT;
            }
        }
        else if (loop_fsm==lDUAL_LEFT) {
            if (core->get_goal_success()) {
                core->setPoseTarget(stairs_poses.loop_dual_left).move(core->get_vel());
                loop_fsm=lSHIFT_FRONT_LEFT;
            }
        }
        else if (loop_fsm==lSHIFT_FRONT_LEFT) {
            if (core->get_goal_success()) {
                core->setPoseTarget(stairs_poses.loop_shift_frontleft).move(core->get_vel()/3);
                if (!core->get_stop()) {
                    loop_fsm=lFWD_RIGHT;
                }
                else {
                    loop_fsm=lFWD_LEFT;
                    stairs_fsm = Stairs_FSM::STOP;
                    stop_fsm=sFWD_RIGHT;
                    if (DEBUG) ROS_INFO("STOP");
                }
            }
        }
        else if (loop_fsm==lLIFT_RIGHT) {
            if (core->get_goal_success()) {
                core->setPoseTarget(stairs_poses.loop_lift_right).move(core->get_vel());
                loop_fsm=lFWD_RIGHT;
            }
        }
        else if (loop_fsm==lFWD_RIGHT) {
            if (core->get_goal_success()) {
                core->setPoseTarget(stairs_poses.loop_fwd_right).move(core->get_vel());
                loop_fsm=lDUAL_RIGHT;
            }
        }
        else if (loop_fsm==lDUAL_RIGHT) {
            if (core->get_goal_success()) {
                core->setPoseTarget(stairs_poses.loop_dual_right).move(core->get_vel());
                loop_fsm=lSHIFT_FRONT_RIGHT;
            }
        }
        else if (loop_fsm==lSHIFT_FRONT_RIGHT) {
            if (core->get_goal_success()) {
                core->setPoseTarget(stairs_poses.loop_shift_frontright).move(core->get_vel()/3);
                loop_fsm=lLIFT_LEFT;
                // Go to stop if control input
                if (core->get_stop()) {
                    stairs_fsm = Stairs_FSM::STOP;
                    if (DEBUG) ROS_INFO("STOP");
                }
            }
        }
    }

    //**********************STOP***********************

    else if (stairs_fsm == Stairs_FSM::STOP) {

        // Stop
        if (stop_fsm==sLIFT_LEFT) {
            if (core->get_goal_success()) {
                core->setPoseTarget(stairs_poses.stop_lift_left).move(core->get_vel());
                stop_fsm=sFWD_LEFT;
            }
        }
        if (stop_fsm==sFWD_LEFT) {
            if (core->get_goal_success()) {
                core->setPoseTarget(stairs_poses.stop_fwd_left).move(core->get_vel());
                stop_fsm=sDEFAULT;
            }
        }
        else if (stop_fsm==sFWD_RIGHT) {
            if (core->get_goal_success()) {
                core->setPoseTarget(stairs_poses.stop_fwd_right).move(core->get_vel());
                stop_fsm=sDEFAULT;
            }
        }
        else if (stop_fsm==sDEFAULT) {
            if (core->get_goal_success()) {;
                core->setPoseTarget(stairs_poses.pose_default).move(core->get_vel());
                stop_fsm=sLIFT_LEFT;
                // Go to Stand
                stairs_fsm = Stairs_FSM::STAND;
                core->set_isstanding(true);
                if (DEBUG) ROS_INFO("STAND");
            }
        }
    }
}

void Stairs::init_StateMachine() {
    // Init to first element in FSM
    stairs_fsm = Stairs_FSM::STOP;
    init_fsm=iSHIFT_LEFT;
    loop_fsm=lLIFT_LEFT;
    stop_fsm=sDEFAULT;
}
