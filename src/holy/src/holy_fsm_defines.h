#ifndef HOLY_MAIN_H
#define HOLY_MAIN_H

// Main Program FSM
enum class Holy_FSM {START,WALK,STAIRS,KINECT,FIGHT, WAIT};

// Kinect FSM
enum class Kinect_FSM {WAIT, LBALANCE, RBALANCE, DUALBALANCE, STOP, INIT};

// Stairs FSM
enum class Stairs_FSM {STAND,INIT,LOOP,STOP};

// Walk FSM
enum class Walk_FSM {STAND,INIT,LOOP,STOP};

// Fight FSM
enum class Fight_FSM{STANCE, WAIT, LOOP};

// Controller buttons
enum class Controller_Button : int {L1 = 10,
                                    L2 = 8,
                                    R1 = 11,
                                    R2 = 9,
                                    DreiEck = 12, Kreis =13, X = 14, VierEck = 15 };

#endif // STAIRS_H


