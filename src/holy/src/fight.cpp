#include "fight.h"

#include "core.h"
#include "poses/poses.h"
#include "poses/parser.h"

#define DEBUG 1

const double Fight::velocity_mult = 10.0;

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

    //velocity = core->get_vel();

    switch(fight_fsm) {
    //************** WAIT FOR BUTTON CLICK ***************
    case Fight_FSM::WAIT:
    {
        if (core->get_buttons()[static_cast<int>(Controller_Button::X)] == 1)
        {
            // reset button
            core->set_buttons(static_cast<int>(Controller_Button::X), 0);

            core->set_isstanding(false);
            loop_fsm = RP_forward;
            fight_fsm_tmp = Fight_FSM::LOOP;
            fight_fsm = Fight_FSM::LOOP;
            if (DEBUG) ROS_INFO("Right Punch");
        }
        else if (core->get_buttons()[static_cast<int>(Controller_Button::Kreis)] == 1)
        {
            // reset button
            core->set_buttons(static_cast<int>(Controller_Button::Kreis), 0);

            core->set_isstanding(false);
            loop_fsm = LP_forward;
            fight_fsm_tmp = Fight_FSM::LOOP;
            fight_fsm = Fight_FSM::LOOP;
            if (DEBUG) ROS_INFO("Left Punch");
        }

    } break;
        //********************** LOOP **********************
    case Fight_FSM::LOOP:
    {
        if (loop_fsm == RP_forward) {
            if (core->get_goal_success()) {
                core->setPoseTarget(fight_poses.fight_punch_right_forward).move(velocity * velocity_mult);
                loop_fsm = RP_sideways;
            }
        }
        else if (loop_fsm == RP_sideways) {
            if (core->get_goal_success()) {
                if (core->get_buttons()[static_cast<int>(Controller_Button::X)] == 1) {
                    core->setPoseTarget(fight_poses.fight_punch_right_sideways).move(velocity * velocity_mult);
                }
                fight_fsm = Fight_FSM::STANCE;
            }
        }
        else if (loop_fsm == LP_forward) {
            if (core->get_goal_success()) {
                core->setPoseTarget(fight_poses.fight_punch_left_forward).move(velocity * velocity_mult);
                loop_fsm = LP_sideways;
            }
        }
        else if (loop_fsm == LP_sideways) {
            if (core->get_goal_success()) {
                if (core->get_buttons()[static_cast<int>(Controller_Button::Kreis)] == 1) {
                    core->setPoseTarget(fight_poses.fight_punch_left_sideways).move(velocity * velocity_mult);
                }
                fight_fsm = Fight_FSM::STANCE;
            }
        }

    } break;
        //****************** GO TO STANCE *******************
    case Fight_FSM::STANCE:
    {
        if (core->get_goal_success()) {
            double vel = velocity;
            if (fight_fsm_tmp == Fight_FSM::LOOP)
                vel *= velocity_mult;
            core->setPoseTarget(fight_poses.fight_stance).move(vel);
            // Go to Stance
            core->set_isstanding(true);
            if (DEBUG) ROS_INFO("Ready to fight");
            fight_fsm_tmp = Fight_FSM::STANCE;
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
    fight_fsm_tmp = Fight_FSM::STANCE;
    loop_fsm = RP_forward;

    velocity = 2.0;
}
