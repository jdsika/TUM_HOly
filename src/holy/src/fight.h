/*  HOly Fight Fight.cpp
 *  Ways to move limbs that make the robot Fight
 *  Laurenz & Simon 2015-05-29
 */

#ifndef _Fight_H_
#define _Fight_H_

#include "poses/poses.h"

class Core;

class Fight {
public:
    Fight(Core* core);
    ~Fight();

    // FSMs
    typedef enum {STAND,RPunch,LPunch} Fight_FSM;

    Fight_FSM fight_fsm;

    // State machine
    void StateMachine();

    void init_StateMachine();


private:
    Core* core;
    // poses object
    Poses fight_poses;

};

#endif
