/*  HOly Walk walk.cpp
 *  A number of poses that the robot can move to.
 *  Laurenz & Simon & Carlo 2015-06-11
 */

#ifndef _POSES_H_
#define _POSES_H_

//#include <map>

#include "robopose.h"
#include <string>
#include <vector>

// The Poses class contains a number of poses that can be used in a state
// machine with the goal of having the robot fulfill its tasks.
inline double d2r(const double degree)
{
    return M_PI*degree/180.0;
}

inline double r2d(const double radian)
{
    return 180.0*radian/M_PI;
}

class Poses
{

private:
    Poses();
    ~Poses();

public:
    static bool printCSVRows(std::string filename);

    static bool parseRoboPositions(std::string filename);

    static bool getWorkingDirectory();

    // Compute Z offset due to weight shifting
    static double compute_Z_off(double angle);

public:
    static const std::string filename;

    // The default pose is the basic pose that must be used as offset for other relatively defined poses
    static RoboPose pose_default;

    static RoboPose shift_toleft;
    static RoboPose shift_toright;
    static RoboPose lift_right;
    static RoboPose lift_left;
    static RoboPose fwd_right;
    static RoboPose fwd_left;
    static RoboPose dual_right;
    static RoboPose dual_left;

    static RoboPose init_shift_toleft;
    static RoboPose init_shift_toright;
    static RoboPose init_lift_left;
    static RoboPose init_lift_right;
    static RoboPose init_fwd_left;
    static RoboPose init_fwd_right;
    static RoboPose init_dual_right;
    static RoboPose init_shift_frontright;
    static RoboPose loop_lift_left;
    static RoboPose loop_fwd_left;
    static RoboPose loop_dual_left;
    static RoboPose loop_shift_frontleft;
    /*// Linken Fuss nach aussen ziehen, Insgesamt nach rechts lehnen durch pitch auf beiden fuessen
    static const RoboPose pose_shift_weight_toright;
    // Linken Fuss anheben und beide fuesse noch weiter pitchen um servo-verbiegung zu kompensieren
    static const RoboPose pose_lift_left_foot;
    static const RoboPose pose_left_foot_advance_forward;
    static const RoboPose pose_left_foot_advanced_down;
    static const RoboPose pose_left_foot_advanced_shiftweighttoleft;

    // Gespiegelt
    static const RoboPose pose_shift_weight_toleft;
    static const RoboPose pose_lift_right_foot;
    static const RoboPose pose_right_foot_advance_forward;
    static const RoboPose pose_right_foot_advanced_down;
    static const RoboPose pose_right_foot_advanced_shiftweighttoright;
    */
    static std::vector<RoboPose> walkingPoses;

};

#endif
