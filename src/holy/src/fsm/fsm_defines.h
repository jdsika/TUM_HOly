#ifndef HOLY_MAIN_H
#define HOLY_MAIN_H

// Main Program FSM
enum class Holy_FSM {START,WALK,STAIRS,KINECT,FIGHT, WAIT};

// Kinect FSM

enum class Kinect_FSM {STAND, LBALANCE, RBALANCE};


// Stairs FSM
enum class Stairs_FSM {WAIT,CLIMB,STOP, CHEER};

// Walk FSM
enum class Walk_FSM {STAND,INIT,LOOP,STOP};


// Fight FSM
enum class Fight_FSM{STANCE, WAIT, LOOP};

// Controller buttons
enum class Controller_Button : int {L1 = 10,
                                    L2 = 8,
                                    R1 = 11,
                                    R2 = 9,
                                    DreiEck = 12, Kreis =13, X = 14, VierEck = 15,
                                    PS = 16};

#endif



