#ifndef HOLY_MAIN_H
#define HOLY_MAIN_H

// Main Program FSM
enum class Holy_FSM {START,WALK,STAIRS,KINECT,FIGHT, WAIT};

// Kinect FSM
enum class Kinect_FSM {STAND, LBALANCE, RBALANCE};

// Stairs FSM
enum class Stairs_FSM {STAND,INIT,LOOP,STOP};

// Walk FSM
enum class Walk_FSM {STAND,INIT,LOOP,STOP};

#endif


