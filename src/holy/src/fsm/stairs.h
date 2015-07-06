#ifndef STAIRS_H
#define STAIRS_H

#include "../poses/poses.h"
#include "fsm_defines.h"

class Core;

class Stairs {
public:
    Stairs(Core* core);

    // FSMs
    typedef enum {iSHIFT_LEFT,
                  iLIFT_RIGHT,
                  iFWD_RIGHT,
                  iDUAL_RIGHT,
                  iSHIFT_RIGHT,
                  iLIFT_LEFT,
                  iFWD_LEFT,
                  /*iADJUST_LEFT_PAD*/} CLIMB_FSM;

    Stairs_FSM stairs_fsm;
    CLIMB_FSM init_fsm;

    // State machine
    void StateMachine();

    void init_StateMachine();

private:
    Core* core;
    // poses object
    Poses stairs_poses;
    double velocity;

};

#endif // STAIRS_H


