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

    //**********************STAND***********************

    if (fight_fsm==STAND) {

        if (!core->get_stop()) {
            //fight_fsm=INIT;
            core->set_isstanding(false);
            if (DEBUG) ROS_INFO("INIT");
        }
    }
}

void Fight::init_StateMachine() {
    // Init to first element in FSM
    fight_fsm=STAND;
}
