/*  HOly Walk walk.cpp
 *  A number of poses that the robot can move to.
 *  Laurenz & Simon & Carlo 2015-06-11
 */

#ifndef _POSES_H_
#define _POSES_H_

#include "robopose.h"
#include "limbpose.h"

#include <vector>


class Poses
{

// R, P, Y, X, Y, Z
// Feet: Toes up(+) down(-) / ankles left(+) right(-) / twist left right / X / Y / Z
// Hands: Arms up down / ? / ? / (X) / (Y) / (Z)

// Invert for L/R:
// Feet:  No / Yes / Yes / Yes / No / No
// Hands: No / ?   / Yes / Yes / No / No


// The Poses class contains a number of poses that can be used in a state
// machine with the goal of having the robot fulfill its tasks.

private:
    double step_length,step_height,turning_angle;

    // Compute Z offset due to weight shifting
    static inline double compute_Z_off(double angle)
    {
        return tan(angle* M_PI / 180.0)*(0.078); // von der Mitte der Gelenke gemessen
    }

public:

    Poses();
    ~Poses();

    void update();

    static inline double d2r(const double degree)
    {
        return M_PI*degree/180.0;
    }

    static inline double r2d(const double radian)
    {
        return 180.0*radian/M_PI;
    }


    // The default pose is the basic pose that must be used as offset for other relatively defined poses
    static RoboPose pose_default;
    static RoboPose pose_default_stairs;
    static RoboPose pose_default_walk;
    static RoboPose pose_relax;

    // Relative poses
    RoboPose shift_toleft;
    RoboPose shift_toright;
    RoboPose lift_right;
    RoboPose lift_left;
    RoboPose turn_right;
    RoboPose turn_left;
    RoboPose fwd_right;
    RoboPose fwd_left;
    RoboPose dual_right;
    RoboPose dual_left;
    RoboPose arms_fwd_left_foot;
    RoboPose arms_fwd_right_foot;
    RoboPose arms_fwd_dual;
    RoboPose arm_left_fwd;
    RoboPose arm_right_fwd;
    RoboPose lean_fwd_right;

    // Carlo compensate for walking
    RoboPose comp_walk_fwd_left;
    RoboPose comp_walk_fwd_right;
    RoboPose comp_shift_to_right;

    // Stairs compensates
    RoboPose comp_stairs_shift_toleft;
    RoboPose comp_stairs_shift_toright;
    RoboPose comp_stairs_left_pad;
    RoboPose comp_stairs_fwd_right;
    RoboPose comp_stairs_dual_right;
    RoboPose comp_stairs_right_down;
    RoboPose comp_stairs_shift_frontright;

    // Absolute poses
    RoboPose init_shift_toleft;
    RoboPose init_lift_right;
    RoboPose init_fwd_right;
    RoboPose init_dual_right;
    RoboPose init_shift_frontright;

    RoboPose loop_lift_left;
    RoboPose loop_fwd_left;
    RoboPose loop_dual_left;
    RoboPose loop_shift_frontleft;
    RoboPose loop_lift_right;
    RoboPose loop_fwd_right;
    RoboPose loop_dual_right;
    RoboPose loop_shift_frontright;
    RoboPose loop_shift_turnright;
    RoboPose loop_shift_turnleft;

    RoboPose stop_lift_left;
    RoboPose stop_fwd_left;
    RoboPose stop_lift_right;
    RoboPose stop_fwd_right;
    // STAIR POSES
    RoboPose stairs_shift_toleft;
    RoboPose stairs_lift_right;
    RoboPose stairs_fwd_right;
    RoboPose stairs_dual_right;
    RoboPose stairs_right_down;
    RoboPose stairs_shift_frontright;
    RoboPose stairs_lean_frontright;
    RoboPose stairs_adjust_left_pad;
    RoboPose stairs_fwd_left;
    RoboPose stairs_jippie;

    // POSES FOR FIGHTING
    RoboPose fight_stance;
    RoboPose fight_punch_right_forward;
    RoboPose fight_punch_left_forward;
    RoboPose fight_punch_right_sideways;
    RoboPose fight_punch_left_sideways;

    void set_step_length(double length);
    double get_step_length();
    void set_step_height(double height);
    void set_turning_angle(double angle);

};

#endif
