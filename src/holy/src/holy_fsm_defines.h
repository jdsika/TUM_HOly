#ifndef HOLY_MAIN_H
#define HOLY_MAIN_H

// Main Program FSM
enum class Holy_FSM {START,WALK,STAIRS,KINECT,FIGHT, WAIT};

// Kinect FSM
<<<<<<< HEAD
enum class Kinect_FSM {STAND, LBALANCE, RBALANCE};
=======
enum class Kinect_FSM {WAIT, LBALANCE, RBALANCE, DUALBALANCE, STOP, INIT};
>>>>>>> 133c01c481cb23e569e6bd4df28fab38085899fe

// Stairs FSM
enum class Stairs_FSM {STAND,INIT,LOOP,STOP};

// Walk FSM
enum class Walk_FSM {STAND,INIT,LOOP,STOP};

<<<<<<< HEAD
#endif
=======
// Fight FSM
enum class Fight_FSM{STANCE, WAIT, LOOP};

// Controller buttons
enum class Controller_Button : int {L1 = 10,
                                    L2 = 8,
                                    R1 = 11,
                                    R2 = 9,
                                    DreiEck = 12, Kreis =13, X = 14, VierEck = 15 };

#endif // STAIRS_H
>>>>>>> 133c01c481cb23e569e6bd4df28fab38085899fe


