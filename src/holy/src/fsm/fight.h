/*  HOly Fight Fight.cpp
 *  Ways to move limbs that make the robot Fight
 *  Laurenz & Simon 2015-05-29
 */

#ifndef _Fight_H_
#define _Fight_H_

#include "../poses/poses.h"
#include "fsm_defines.h"

class Core;

class Fight {
public:
    Fight(Core* core);

    Fight_FSM fight_fsm;
    Fight_FSM fight_fsm_tmp;

    // State machine
    void StateMachine();

    void init_StateMachine();

    typedef enum {RP_forward, RP_sideways, LP_forward, LP_sideways} LOOP_FSM;


private:
    Core* core;
    // poses object
    Poses fight_poses;
    LOOP_FSM loop_fsm;

    double velocity;
    const static double velocity_mult;
};

#endif
