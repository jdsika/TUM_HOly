/*  HOly Walk walk.cpp
 *  Ways to move limbs that make the robot walk
 *  Laurenz & Simon 2015-05-29
 */

#ifndef _WALK_H_
#define _WALK_H_

class Core;

class Walk {
public:
    Walk(Core* core);
    ~Walk();

    void executeStateMachine();

    // FSMs
    typedef enum {STAND,INIT,LOOP,STOP} Walk_FSM;
    Walk_FSM walk_fsm;

private:
    Core* core;

};

#endif
