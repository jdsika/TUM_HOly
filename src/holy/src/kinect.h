/*  HOly KINECT KINECT.cpp
 *  Ways to move limbs that make the robot KINECT
 *  Laurenz & Simon 2015-05-29
 */

#ifndef _KINECT_H_
#define _KINECT_H_

#include "poses/poses.h"

class Core;

class Kinect {
public:
    Kinect(Core* core);
    ~Kinect();

    void executeStateMachine();

    // FSMs
    typedef enum {STAND, LBALANCE, RBALANCE, DUALBALANCE} Kinect_FSM;

    Kinect_FSM kinect_fsm;

    // State machine
    void StateMachine();

    void init_StateMachine();


private:
    Core* core;
    // poses object
    Poses kinect_poses;

};

#endif