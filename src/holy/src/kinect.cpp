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

    if (kinect_fsm == Kinect_FSM::STAND) {

        if (!core->get_stop()) {
            //kinect_fsm=INIT;
            core->set_isstanding(false);
            if (DEBUG) ROS_INFO("INIT");
        }
    }
    else if (kinect_fsm == Kinect_FSM::STOP) {

    }
}

void Kinect::init_StateMachine() {
    // Init to first element in FSM
    kinect_fsm = Kinect_FSM::STAND;
}
