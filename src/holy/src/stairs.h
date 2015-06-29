#ifndef STAIRS_H
#define STAIRS_H

#include "poses/poses.h"
#include "holy_fsm_defines.h"

class Core;

class Stairs {
public:
    Stairs(Core* core);
    ~Stairs();

    // FSMs
    typedef enum {iSHIFT_LEFT,iLIFT_RIGHT,iFWD_RIGHT,iDUAL_RIGHT,iFWD_LEFT,iADJUST_LEFT_PAD,iLEAN_FWD_RIGHT,iRIGHT_DOWN,iSHIFT_FRONT_RIGHT, iJIPPIE} INIT_FSM;
    typedef enum {lLIFT_LEFT,lFWD_LEFT,lDUAL_LEFT,lSHIFT_FRONT_LEFT,lLIFT_RIGHT,lFWD_RIGHT,lDUAL_RIGHT,lSHIFT_FRONT_RIGHT} LOOP_FSM;
    typedef enum {sLIFT_LEFT,sFWD_LEFT,sLIFT_RIGHT,sFWD_RIGHT,sDEFAULT} STOP_FSM;

    Stairs_FSM stairs_fsm;
    INIT_FSM init_fsm;
    LOOP_FSM loop_fsm;
    STOP_FSM stop_fsm;

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


