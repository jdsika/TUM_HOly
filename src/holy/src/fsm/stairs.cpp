#include "stairs.h"

#include "../core/core.h"
#include "../poses/poses.h"
#include "../poses/parser.h"

#define DEBUG 1

Stairs::Stairs(Core *core) : core{core}
{

    init_StateMachine();
    velocity=1.0;

}

void Stairs::StateMachine() {

    // update parameters
    stairs_poses.set_step_height(0.050); // max 0.05
    stairs_poses.set_step_length(0.075); // max 0.033
    stairs_poses.update();
    //**********************STAND***********************

    switch(stairs_fsm) {
    case Stairs_FSM::WAIT:
    {

        if (!core->get_stop()) {
            stairs_fsm = Stairs_FSM::CLIMB;
            init_fsm = iSHIFT_LEFT;
            core->set_isstanding(false);
            if (DEBUG) ROS_INFO("Climb one stair");
        }
        else if (core->get_buttons()[static_cast<int>(Controller_Button::Kreis)]) {
            stairs_fsm = Stairs_FSM::CHEER;
            core->set_isstanding(false);
            if (DEBUG) ROS_INFO("Cheer!!!");
        }
    } break;

        //**********************Climb one stair **********************

    case Stairs_FSM::CLIMB:
    {
        ros::Duration(0.5).sleep();

        //Init
        if (init_fsm==iSHIFT_LEFT) {
            if (core->get_goal_success()) {
                core->setPoseTarget(stairs_poses.stairs_shift_toleft).move(velocity);
                init_fsm=iLIFT_RIGHT;
            }
        }
        else if (init_fsm==iLIFT_RIGHT) {
            if (core->get_goal_success()) {
                core->setPoseTarget(stairs_poses.stairs_lift_right).move(velocity);
                init_fsm=iFWD_RIGHT;
            }
        }
        else if (init_fsm==iFWD_RIGHT) {
            if (core->get_goal_success()) {
                core->setPoseTarget(stairs_poses.stairs_fwd_right).move(velocity);
                init_fsm=iDUAL_RIGHT;
            }
        }
        else if (init_fsm==iDUAL_RIGHT) {
            if (core->get_goal_success()) {
                core->setPoseTarget(stairs_poses.stairs_dual_right).move(velocity);
                init_fsm=iRIGHT_DOWN;
            }
        }
        else if (init_fsm==iRIGHT_DOWN) {
            if (core->get_goal_success()) {
                core->setPoseTarget(stairs_poses.stairs_right_down).move(velocity);
                init_fsm=iSHIFT_FRONT_RIGHT;
            }
        }
        else if (init_fsm==iSHIFT_FRONT_RIGHT) {
            if (core->get_goal_success()) {
                core->setPoseTarget(stairs_poses.stairs_shift_frontright).move(velocity);
                init_fsm=iLEAN_FWD_RIGHT;
            }
        }
        else if (init_fsm==iLEAN_FWD_RIGHT) {
            if (core->get_goal_success()) {
                core->setPoseTarget(stairs_poses.stairs_lean_frontright).move(velocity);
                init_StateMachine();
            }
        }
        //        else if (init_fsm==iADJUST_LEFT_PAD) {
        //            if (core->get_goal_success()) {
        //                core->setPoseTarget(stairs_poses.stairs_adjust_left_pad).move(velocity);
        //                init_fsm=iJIPPIE;
        //            }
        //        }

    } break;

        //**********************STOP***********************

    case Stairs_FSM::STOP:
    {
        if (core->get_goal_success()) {;
            core->setPoseTarget(stairs_poses.pose_default).move(velocity);
            init_StateMachine();
            // Go to Stand
            stairs_fsm = Stairs_FSM::WAIT;
            core->set_isstanding(true);
            if (DEBUG) ROS_INFO("Wait: LJoy = climb / Circle = cheer");
        }
    } break;
    case Stairs_FSM::CHEER:
    {
        if (core->get_goal_success()) {
            core->setPoseTarget(stairs_poses.stairs_jippie).move(velocity*10);
            init_StateMachine();
        }

    } break;
    default:
    {
        ROS_INFO("Unknown stair state");
    }
    }
}

void Stairs::init_StateMachine() {
    // Init to first element in FSM
    stairs_fsm = Stairs_FSM::STOP;
    init_fsm = iSHIFT_LEFT;
}
