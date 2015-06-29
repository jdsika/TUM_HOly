#include "stairs.h"

#include "core.h"
#include "poses/poses.h"
#include "poses/parser.h"

#define DEBUG 1

Stairs::Stairs(Core *core) : core{core}
{

    init_StateMachine();
    velocity=1.0;

}

Stairs::~Stairs()
{

}

void Stairs::StateMachine() {

     // update parameters
     stairs_poses.set_step_height(0.050); // max 0.05
     stairs_poses.set_step_length(0.075); // max 0.033
     stairs_poses.update();
    //**********************STAND***********************

    if (stairs_fsm == Stairs_FSM::STAND) {

        if (!core->get_stop()) {
            stairs_fsm = Stairs_FSM::INIT;
            init_fsm = iSHIFT_LEFT;
            core->set_isstanding(false);
            if (DEBUG) ROS_INFO("INIT");
        }
        else if (core->get_buttons()[static_cast<int>(Controller_Button::Kreis)]) {
            stairs_fsm = Stairs_FSM::INIT;
            init_fsm = iJIPPIE;
        }
    }

    //**********************INIT**********************

    else if (stairs_fsm == Stairs_FSM::INIT) {
        ros::Duration(0.5).sleep();

        /*if(core->get_goal_success())
        {
            ROS_INFO("wait for <X>");
            while(ros::ok() && !core->get_buttons()[static_cast<int>(Controller_Button::X)])
            {
                ros::Duration(0.1).sleep();
            }
            ROS_INFO("continue.");
        }*/

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
                init_fsm=iSHIFT_LEFT;
                stairs_fsm=Stairs_FSM::STOP;
                stop_fsm=sDEFAULT;
            }
        }
        else if (init_fsm==iADJUST_LEFT_PAD) {
            if (core->get_goal_success()) {
                core->setPoseTarget(stairs_poses.stairs_adjust_left_pad).move(velocity);
                init_fsm=iJIPPIE;
            }
        }
        else if (init_fsm==iJIPPIE) {
            if (core->get_goal_success()) {
                core->setPoseTarget(stairs_poses.stairs_jippie).move(velocity*10);
                init_fsm=iSHIFT_LEFT;
                stairs_fsm=Stairs_FSM::STOP;
                stop_fsm=sDEFAULT;
            }
        }
    }

    //**********************STOP***********************

    else if (stairs_fsm == Stairs_FSM::STOP) {

        if (stop_fsm == sDEFAULT) {
            if (core->get_goal_success()) {;
                core->setPoseTarget(stairs_poses.pose_default).move(velocity);
                init_StateMachine();
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
    init_fsm = iSHIFT_LEFT;
    loop_fsm = lLIFT_LEFT;
    stop_fsm = sDEFAULT;
}
