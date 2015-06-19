/*  HOly Walk walk.cpp
 *  A number of poses that the robot can move to.
 *  Laurenz & Simon & Carlo 2015-06-11
 */

#ifndef _POSES_H_
#define _POSES_H_

#include "robopose.h"
#include "limbpose.h"

#include <vector>


namespace Poses
{

// R, P, Y, X, Y, Z
// Feet: Toes up(+) down(-) / ankles left(+) right(-) / twist left right / X / Y / Z
// Hands: Arms up down / ? / ? / (X) / (Y) / (Z)

// Invert for L/R:
// Feet:  No / Yes / Yes / Yes / No / No
// Hands: No / ?   / Yes / Yes / No / No


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

// Compute Z offset due to weight shifting
static inline double compute_Z_off(double angle)
{
    return tan(angle* M_PI / 180.0)*(0.078); // von der Mitte der Gelenke gemessen
}



// The default pose is the basic pose that must be used as offset for other relatively defined poses
static const RoboPose pose_default(std::vector<LimbPose> {
                       //changed to have lower arms
                                       LimbPose(Core::Limb::LEFT_HAND,  d2r(90),  0, d2r(90-1), -0.15, 0, 0.03),
                                       LimbPose(Core::Limb::RIGHT_HAND, d2r(90),  0, d2r(-90+1),  0.15, 0, 0.03),
                                       LimbPose(Core::Limb::LEFT_FOOT,  d2r(2),  0,  0, -0.03, 0.0, -0.2),
                                       LimbPose(Core::Limb::RIGHT_FOOT, d2r(2),  0,  0,  0.03, 0.0, -0.2)
                                   } , "pose_default");



// General relative poses, easy to change them all at once, do not call them seperate
static const RoboPose shift_toleft =
        RoboPose( std::vector<LimbPose> {
                        LimbPose(Core::Limb::LEFT_HAND,  d2r(30),  0, 0, 0, 0, 0),
                        LimbPose(Core::Limb::RIGHT_HAND, d2r(-70),  0, 0,  0, 0, 0),
                        LimbPose (Core::Limb::LEFT_FOOT,  d2r(0), d2r(14), d2r(0), 0, 0, 0),
                        LimbPose (Core::Limb::RIGHT_FOOT, d2r(0), d2r(14), d2r(0), 0, 0, compute_Z_off(14)*-0.5),
                    }, "shift_toleft");

static const RoboPose shift_toright =
        RoboPose( std::vector<LimbPose> {
                        LimbPose(Core::Limb::LEFT_HAND,  d2r(-70),  0, 0, 0, 0, 0),
                        LimbPose(Core::Limb::RIGHT_HAND, d2r(30),  0, 0,  0, 0, 0),
                        LimbPose (Core::Limb::LEFT_FOOT,  d2r(0), d2r(-13), d2r(0), 0, 0, compute_Z_off(13)*-0.5),
                        LimbPose (Core::Limb::RIGHT_FOOT, d2r(0), d2r(-13), d2r(0), 0, 0, 0),
                    }, "shift_toright");

static const RoboPose lift_right =
        RoboPose( std::vector<LimbPose> {
                        LimbPose (Core::Limb::RIGHT_FOOT,  d2r(0), d2r(0), d2r(0), 0.02, 0,0.02), // Compensate x due to weak motors
                    }, "lift_right");

static const RoboPose lift_left =
        RoboPose( std::vector<LimbPose> {
                        LimbPose (Core::Limb::LEFT_FOOT,  d2r(0), d2r(0), d2r(0), -0.02, 0,0.02),
                    }, "lift_left");

static const RoboPose fwd_left =
        RoboPose( std::vector<LimbPose> {
                        LimbPose (Core::Limb::LEFT_FOOT,  d2r(0), d2r(0), d2r(0), 0, 0.02,0),
                        LimbPose (Core::Limb::RIGHT_FOOT,  d2r(0), d2r(0), d2r(0), 0, 0,0),
                    }, "fwd_left");

static const RoboPose fwd_right =
        RoboPose( std::vector<LimbPose> {
                        LimbPose (Core::Limb::RIGHT_FOOT,  d2r(0), d2r(0), d2r(0), 0, 0.02,0),
                        LimbPose (Core::Limb::LEFT_FOOT,  d2r(0), d2r(0), d2r(0), 0, 0,0),
                    }, "fwd_left");

static const RoboPose dual_right =
        RoboPose( std::vector<LimbPose> {
                      LimbPose (Core::Limb::LEFT_FOOT,  d2r(0), d2r(0), d2r(0), 0, -Poses::fwd_right.getLimb(Core::Limb::RIGHT_FOOT).default_y/2, 0),
                      LimbPose (Core::Limb::RIGHT_FOOT, d2r(0), d2r(0), d2r(0), 0, -Poses::fwd_right.getLimb(Core::Limb::RIGHT_FOOT).default_y/2, 0),
                  }, "dual_right") ;

static const RoboPose dual_left =
        RoboPose( std::vector<LimbPose> {
                      LimbPose (Core::Limb::LEFT_FOOT,  d2r(0), d2r(0), d2r(0), 0, -Poses::fwd_right.getLimb(Core::Limb::LEFT_FOOT).default_y/2, 0),
                      LimbPose (Core::Limb::RIGHT_FOOT, d2r(0), d2r(0), d2r(0), 0, -Poses::fwd_right.getLimb(Core::Limb::LEFT_FOOT).default_y/2, 0),
                  }, "dual_left") ;

// Absolute poses:
// Init
static const RoboPose init_shift_toleft = Poses::pose_default+Poses::shift_toleft;
static const RoboPose init_shift_toright = Poses::pose_default+Poses::shift_toright;
static const RoboPose init_lift_left = Poses::init_shift_toright+Poses::lift_left;
static const RoboPose init_lift_right = Poses::init_shift_toleft+Poses::lift_right;
static const RoboPose init_fwd_left = Poses::init_lift_left+Poses::fwd_left;
static const RoboPose init_fwd_right = Poses::init_lift_right+Poses::fwd_right;
static const RoboPose init_dual_right = Poses::init_fwd_right+ Poses::dual_right- Poses::shift_toleft - Poses::lift_right;
static const RoboPose init_shift_frontright = Poses::init_dual_right+ Poses::shift_toright + Poses::dual_right ;

// Loop
static const RoboPose loop_lift_left = Poses::init_shift_frontright+Poses::lift_left;
static const RoboPose loop_fwd_left = Poses::loop_lift_left+Poses::fwd_left+Poses::fwd_left;
static const RoboPose loop_dual_left = Poses::loop_fwd_left+ Poses::dual_left- Poses::shift_toright - Poses::lift_left;
static const RoboPose loop_shift_frontleft = Poses::loop_dual_left+ Poses::shift_toleft + Poses::dual_left ;
static const RoboPose loop_lift_right = Poses::loop_shift_frontleft+Poses::lift_right;
static const RoboPose loop_fwd_right = Poses::loop_lift_right+Poses::fwd_right+Poses::fwd_right;
static const RoboPose loop_dual_right = Poses::loop_fwd_right+ Poses::dual_right- Poses::shift_toleft - Poses::lift_right;
static const RoboPose loop_shift_frontright = Poses::loop_dual_right + Poses::shift_toright + Poses::dual_right;

// Stop
static const RoboPose stop_lift_left = Poses::init_shift_frontright+Poses::lift_left;
static const RoboPose stop_fwd_left = Poses::stop_lift_left+Poses::fwd_left;

}

#endif
