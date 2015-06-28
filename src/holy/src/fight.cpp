#include "fight.h"

#include "core.h"
#include "poses/poses.h"
#include "poses/parser.h"

#define DEBUG 1

Fight::Fight(Core *core) : core{core}
{
    init_StateMachine();
}

Fight::~Fight()
{

}


void Fight::StateMachine() {

    // update parameters
    fight_poses.set_step_height(0.01); // max 0.05
    fight_poses.set_turning_angle(core->get_turning_angle());
    fight_poses.set_step_length(core->getStep_length()); // max 0.033
    fight_poses.update();

    switch(fight_fsm) {
    //************** WAIT FOR BUTTON CLICK ***************
    case Fight_FSM::WAIT:
    {
        if (core->get_buttons()[static_cast<int>(Controller_Button::X)] == 1)
        {
            core->set_isstanding(false);
            loop_fsm = RP_forward;
            fight_fsm = Fight_FSM::LOOP;
            if (DEBUG) ROS_INFO("Right Punch");
        }
        else if (core->get_buttons()[static_cast<int>(Controller_Button::Kreis)] == 1)
        {
            core->set_isstanding(false);
            loop_fsm = LP_forward;
            fight_fsm = Fight_FSM::LOOP;
            if (DEBUG) ROS_INFO("Left Punch");
        }

    } break;
        //********************** LOOP **********************
    case Fight_FSM::LOOP:
    {
        if (loop_fsm == RP_forward) {
            if (core->get_goal_success()) {
                core->setPoseTarget(fight_poses.fight_punch_right_forward).move(core->get_vel());
                fight_fsm = Fight_FSM::STANCE;
            }
        }
        else if (loop_fsm == LP_forward) {
            if (core->get_goal_success()) {
                core->setPoseTarget(fight_poses.fight_punch_left_forward).move(core->get_vel());
                fight_fsm = Fight_FSM::STANCE;
            }
        }

    } break;
        //****************** GO TO STANCE *******************
    case Fight_FSM::STANCE:
    {
        if (core->get_goal_success()) {
            core->setPoseTarget(fight_poses.fight_stance).move(core->get_vel());
            // Go to Stance
            core->set_isstanding(true);
            if (DEBUG) ROS_INFO("Ready to fight");
            fight_fsm = Fight_FSM::WAIT;
        }

    } break;
        //******************** UNKNOWN ***********************
    default:
    {
        ROS_INFO("Unknown Fighting State");
    }
    }
}

void Fight::init_StateMachine() {
    // Init to first element in FSM
    fight_fsm = Fight_FSM::STANCE;
    loop_fsm = RP_forward;
}
