#include "kinect.h"

#include "core.h"
#include "poses/poses.h"
#include "poses/parser.h"

#define DEBUG 1

Kinect::Kinect(Core *core) : core{core}
{

    init_StateMachine();

}

Kinect::~Kinect()
{

}


void Kinect::StateMachine() {

    // update parameters
    kinect_poses.set_step_height(0.01); // max 0.05
    kinect_poses.set_turning_angle(core->get_turning_angle());
    kinect_poses.set_step_length(core->getStep_length()); // max 0.033
    kinect_poses.update();

    //**********************STAND***********************

    if (kinect_fsm == Kinect_FSM::WAIT) {

        if (core->get_buttons()[static_cast<int>(Controller_Button::DreiEck)] == 1) {
            kinect_fsm = Kinect_FSM::INIT;
            core->set_isstanding(false);
            if (DEBUG) ROS_INFO("INIT");
        }
    }
    else if (kinect_fsm == Kinect_FSM::INIT) {
        kinect_fsm = Kinect_FSM::STOP;
    }
    else if (kinect_fsm == Kinect_FSM::STOP) {
        if (core->get_goal_success()) {
            core->setPoseTarget(kinect_poses.pose_default).move(core->get_vel());
            // Go to Stand
            core->set_isstanding(true);
            if (DEBUG) ROS_INFO("STAND");
            kinect_fsm = Kinect_FSM::WAIT;
        }

    }
}

void Kinect::init_StateMachine() {
    // Init to first element in FSM
    kinect_fsm = Kinect_FSM::STOP;
}
