#include "poses.h"

#include "limbpose.h"
#include "../core.h"

// R, P, Y, X, Y, Z
// Feet: Toes up(+) down(-) / ankles left(+) right(-) / twist left right / X / Y / Z
// Hands: Arms up down / ? / ? / (X) / (Y) / (Z)

// Invert for L/R:
// Feet:  No / Yes / Yes / Yes / No / No
// Hands: No / ?   / Yes / Yes / No / No

void Poses::update() {
    // update Poses according to new parameters

    pose_default = RoboPose (std::vector<LimbPose> {
                           //changed to have lower arms
                                           LimbPose(Core::Limb::LEFT_HAND,  d2r(10),  0, d2r(90-1), -0.15, 0, 0.03),
                                           LimbPose(Core::Limb::RIGHT_HAND, d2r(10),  0, d2r(-90+1),  0.15, 0, 0.03),
                                           LimbPose(Core::Limb::LEFT_FOOT,  d2r(0),  0,  0, -0.03, 0.00, -0.2),
                                           LimbPose(Core::Limb::RIGHT_FOOT, d2r(0),  0,  0,  0.03, 0.00, -0.2)
                                       } , "pose_default");



    // General relative poses, easy to change them all at once, do not call them seperate
    shift_toleft =
            RoboPose( std::vector<LimbPose> {
                            LimbPose (Core::Limb::LEFT_FOOT,  d2r(0), d2r(14), d2r(0), 0, 0, 0),
                            LimbPose (Core::Limb::RIGHT_FOOT, d2r(0), d2r(14), d2r(0), 0, 0, compute_Z_off(14)*-0.5),
                        }, "shift_toleft");

    shift_toright =
            RoboPose( std::vector<LimbPose> {
                            LimbPose (Core::Limb::LEFT_FOOT,  d2r(0), d2r(-13), d2r(0), 0, 0, compute_Z_off(13)*-0.5),
                            LimbPose (Core::Limb::RIGHT_FOOT, d2r(0), d2r(-13), d2r(0), 0, 0, 0),
                        }, "shift_toright");

    lift_right =
            RoboPose( std::vector<LimbPose> {
                            LimbPose (Core::Limb::RIGHT_FOOT,  d2r(0), d2r(0), d2r(0), 0.01, 0,step_height), // Compensate x due to weak motors
                        }, "lift_right");

    lift_left =
            RoboPose( std::vector<LimbPose> {
                            LimbPose (Core::Limb::LEFT_FOOT,  d2r(0), d2r(0), d2r(0), -0.01, 0,step_height),
                        }, "lift_left");

    turn_right =
            RoboPose( std::vector<LimbPose> {
                            LimbPose (Core::Limb::RIGHT_FOOT,  d2r(0), d2r(0), d2r(turning_angle), fabs(0.01*(turning_angle/20.0)), 0,0), // Compensate x due to weak motors
                        }, "lift_right");

    turn_left =
            RoboPose( std::vector<LimbPose> {
                            LimbPose (Core::Limb::LEFT_FOOT,  d2r(0), d2r(0), d2r(turning_angle), -fabs(0.01*(turning_angle/20.0)), 0,0),
                        }, "lift_left");

    fwd_left =
            RoboPose( std::vector<LimbPose> {
                            LimbPose (Core::Limb::LEFT_FOOT,  d2r(0), d2r(0), d2r(0), 0, step_length,0),
                            LimbPose (Core::Limb::RIGHT_FOOT,  d2r(0), d2r(0), d2r(0), 0, 0,0),
                        }, "fwd_left");

    fwd_right =
            RoboPose( std::vector<LimbPose> {
                            LimbPose (Core::Limb::RIGHT_FOOT,  d2r(0), d2r(0), d2r(0), 0, step_length,0),
                            LimbPose (Core::Limb::LEFT_FOOT,  d2r(0), d2r(0), d2r(0), 0, 0,0),
                        }, "fwd_left");

    dual_right =
            RoboPose( std::vector<LimbPose> {
                          LimbPose (Core::Limb::LEFT_FOOT,  d2r(0), d2r(0), d2r(0), 0, -step_length/2, 0),
                          LimbPose (Core::Limb::RIGHT_FOOT, d2r(0), d2r(0), d2r(0), 0, -step_length/2, 0),
                      }, "dual_right") ;

    dual_left =
            RoboPose( std::vector<LimbPose> {
                          LimbPose (Core::Limb::LEFT_FOOT,  d2r(0), d2r(0), d2r(0), 0, -step_length/2, 0),
                          LimbPose (Core::Limb::RIGHT_FOOT, d2r(0), d2r(0), d2r(0), 0, -step_length/2, 0),
                      }, "dual_left") ;

    arms_fwd_left_foot =
            RoboPose( std::vector<LimbPose> {
                            LimbPose(Core::Limb::LEFT_HAND,  d2r(0),  0, 0, 0, 0, 0),
                            LimbPose(Core::Limb::RIGHT_HAND, d2r(70),  0, 0,  0, 0, 0),
                        }, "arms_fwd_left_foot");

    arms_fwd_right_foot =
            RoboPose( std::vector<LimbPose> {
                            LimbPose(Core::Limb::LEFT_HAND,  d2r(70),  0, 0, 0, 0, 0),
                            LimbPose(Core::Limb::RIGHT_HAND, d2r(0),  0, 0,  0, 0, 0),
                        }, "arms_fwd_right_foot");




    // Absolute poses:
    // Init
    init_shift_toleft = pose_default+shift_toleft+arms_fwd_right_foot;
    init_lift_right = init_shift_toleft+lift_right+turn_right;
    init_fwd_right = init_lift_right+fwd_right;
    init_dual_right = init_fwd_right+ dual_right- shift_toleft - lift_right -arms_fwd_right_foot;
    init_shift_frontright = init_dual_right+ shift_toright + dual_right-turn_right+arms_fwd_left_foot;

    // Loop
    loop_lift_left = init_shift_frontright+lift_left+turn_left;
    loop_fwd_left = loop_lift_left+fwd_left+fwd_left;
    loop_dual_left = loop_fwd_left+ dual_left- shift_toright - lift_left -arms_fwd_left_foot;
    loop_shift_frontleft = loop_dual_left+ shift_toleft + dual_left-turn_left + arms_fwd_right_foot;
    loop_lift_right = loop_shift_frontleft+lift_right+turn_right;
    loop_fwd_right = loop_lift_right+fwd_right+fwd_right;
    loop_dual_right = loop_fwd_right+ dual_right- shift_toleft - lift_right - arms_fwd_right_foot;
    loop_shift_frontright = loop_dual_right + shift_toright + dual_right-turn_right + arms_fwd_left_foot;

    // Stop
    stop_lift_left = loop_shift_frontright+lift_left;
    stop_fwd_left = stop_lift_left+fwd_left;
    stop_lift_right = loop_shift_frontleft+lift_right;
    stop_fwd_right= stop_lift_right+fwd_right;

}

void Poses::set_step_length(double length) {
    step_length=length;
}

void Poses::set_step_height(double height) {
    step_height=height;
}

void Poses::set_turning_angle(double angle) {
    turning_angle=angle;
}

Poses::Poses()
{
    step_length=0.02;
    step_height=0.02;
    turning_angle=0.0;
    update();
}

Poses::~Poses()
{

}
