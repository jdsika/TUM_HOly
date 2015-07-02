#include "poses.h"

#include "limbpose.h"
#include "../core/core.h"

// R, P, Y, X, Y, Z
// Feet: Toes up(+) down(-) / ankles left(+) right(-) / twist left right / X / Y / Z
// Hands: Arms up down / ? / ? / (X) / (Y) / (Z)

// Invert for L/R:
// Feet:  No / Yes / Yes / Yes / No / No
// Hands: No / ?   / Yes / Yes / No / No

RoboPose Poses::pose_default = RoboPose (std::vector<LimbPose> {
                                             //changed to have lower arms
                                             LimbPose(Core::Limb::LEFT_HAND,  d2r(0),  d2r(10), d2r(-1),  0, 0, 0),
                                             LimbPose(Core::Limb::RIGHT_HAND, d2r(0),  d2r(-15), d2r(1),  0, 0, 0),
                                             LimbPose(Core::Limb::LEFT_FOOT,  d2r(5),  0,  0, -0.03, 0.02, -0.2),
                                             LimbPose(Core::Limb::RIGHT_FOOT, d2r(5),  0,  0,  0.03, 0.02, -0.2)
                                         } , "pose_default");

RoboPose Poses::pose_default_stairs = RoboPose (std::vector<LimbPose> {
                                             //changed to have lower arms
                                             LimbPose(Core::Limb::LEFT_HAND,  d2r(10),  0, d2r(90-1), -0.15, 0, 0.03),
                                             LimbPose(Core::Limb::RIGHT_HAND, d2r(10),  0, d2r(-90+1),  0.15, 0, 0.03),
                                             LimbPose(Core::Limb::LEFT_FOOT,  d2r(5),  0,  0, -0.03, 0.02, -0.2),
                                             LimbPose(Core::Limb::RIGHT_FOOT, d2r(5),  0,  0,  0.03, 0.02, -0.2)
                                         } , "pose_default_stairs");

RoboPose Poses::pose_relax = RoboPose (std::vector<LimbPose> {
                                             //changed to have lower arms
                                             LimbPose(Core::Limb::LEFT_HAND,  d2r(0),  d2r(10), d2r(-1),  0, 0, 0),
                                             LimbPose(Core::Limb::RIGHT_HAND, d2r(0),  d2r(-15), d2r(1),  0, 0, 0),
                                             LimbPose(Core::Limb::LEFT_FOOT,  d2r(0),  0,  0, -0.03, 0.00, -0.215),
                                             LimbPose(Core::Limb::RIGHT_FOOT, d2r(0),  0,  0,  0.03, 0.00, -0.215)
                                         } , "pose_relax");

void Poses::update() {


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
                            LimbPose (Core::Limb::LEFT_FOOT,  d2r(0), d2r(0), d2r(0), -0.01, 0,step_height), // Compensate x due to weak motors
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
                          LimbPose (Core::Limb::LEFT_FOOT,  d2r(0), d2r(0), d2r(0), 0, -step_length/2.0, 0),
                          LimbPose (Core::Limb::RIGHT_FOOT, d2r(0), d2r(0), d2r(0), 0, -step_length/2.0, 0),
                      }, "dual_right") ;


    dual_left =
            RoboPose( std::vector<LimbPose> {
                          LimbPose (Core::Limb::LEFT_FOOT,  d2r(0), d2r(0), d2r(0), 0, -step_length/2.0, 0),
                          LimbPose (Core::Limb::RIGHT_FOOT, d2r(0), d2r(0), d2r(0), 0, -step_length/2.0, 0),
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

    arm_left_fwd =
            RoboPose( std::vector<LimbPose> {
                            LimbPose(Core::Limb::LEFT_HAND,  d2r(0),  d2r(-90), d2r(0), 0, 0, 0),
                        }, "arm_left_fwd");

    arm_right_fwd =
            RoboPose( std::vector<LimbPose> {
                            LimbPose(Core::Limb::RIGHT_HAND,  d2r(0), d2r(90), d2r(-45), 0, 0, 0),
                        }, "arm_right_fwd");

    lean_fwd_right =
            RoboPose( std::vector<LimbPose> {
                            LimbPose (Core::Limb::RIGHT_FOOT,  d2r(0), d2r(0), d2r(0), 0, -0.01, -0.00)
                        }, "lean_fwd_right");


    // Extra poses for walking
    comp_walk_fwd_left =    // PERFECT
            RoboPose( std::vector<LimbPose> {
                            LimbPose (Core::Limb::LEFT_FOOT,  d2r(-4), d2r(10), d2r(0), -0.001, 0.00, 0.00),
                            LimbPose (Core::Limb::RIGHT_FOOT, d2r(0), d2r(0), d2r(0), 0.00, 0.00, 0.00),
                        }, "comp_walk_dual_right");

    // RIGHT FOOT:  - roll = toes up // +pitch = auf aussenkante drehen

    comp_walk_fwd_right =
            RoboPose( std::vector<LimbPose> {
                            LimbPose (Core::Limb::LEFT_FOOT,  d2r(0), d2r(0), d2r(1), -0.001, 0.00, 0.00),
                            LimbPose (Core::Limb::RIGHT_FOOT, d2r(4), d2r(-10), d2r(0), 0.001, 0.00, 0.00),
                        }, "comp_walk_fwd_right");

    // Extra poses to compensate the stair climbing
    comp_stairs_dual_right =
            RoboPose( std::vector<LimbPose> {
                            LimbPose (Core::Limb::LEFT_FOOT,  d2r(0), d2r(0), d2r(0), 0, 0, -0.01),
                            LimbPose (Core::Limb::RIGHT_FOOT, d2r(0), d2r(0), d2r(0), 0.00, 0, 0),
                        }, "comp_stairs_dual_right");
    comp_stairs_fwd_right =
            RoboPose( std::vector<LimbPose> {
                            LimbPose (Core::Limb::RIGHT_FOOT,  d2r(0), d2r(0), d2r(0), 0, 0,0),
                            LimbPose (Core::Limb::LEFT_FOOT,  d2r(-3), d2r(0), d2r(0), 0, 0,0),
                        }, "comp_stairs_fwd_right");
    comp_stairs_shift_toleft =
            RoboPose( std::vector<LimbPose> {
                            LimbPose (Core::Limb::RIGHT_FOOT, d2r(0), d2r(0), d2r(0), 0.01, 0, 0),
                            LimbPose (Core::Limb::LEFT_FOOT,  d2r(0), d2r(8), d2r(0), -0.02, 0, -0.00),
                        }, "comp_stairs_shift_toleft");
    comp_stairs_shift_toright =
            RoboPose( std::vector<LimbPose> {
                            LimbPose (Core::Limb::RIGHT_FOOT, d2r(0), d2r(-4), d2r(0), -0.015, 0, 0.00),
                            LimbPose (Core::Limb::LEFT_FOOT,  d2r(-10), d2r(0), d2r(0), 0.01, 0, -0.00),
                        }, "comp_stairs_shift_toleft");
    comp_stairs_right_down =
            RoboPose( std::vector<LimbPose> {
                          LimbPose (Core::Limb::LEFT_FOOT,  d2r(-40), d2r(0), d2r(0), 0, -0.03, +0.008),
                          LimbPose (Core::Limb::RIGHT_FOOT, d2r(18), d2r(0), d2r(0), 0, 0, -0.004)
                        }, "comp_stairs_right_down");
    comp_stairs_left_pad =
            RoboPose( std::vector<LimbPose> {
                          LimbPose (Core::Limb::RIGHT_FOOT, d2r(-6), d2r(0), d2r(0), 0, 0, 0.00),
                          LimbPose (Core::Limb::LEFT_FOOT,  d2r(46), d2r(0), d2r(0), 0, 0, -0.01)
                        }, "comp_stairs_right_down");
    comp_stairs_shift_frontright =
            RoboPose( std::vector<LimbPose> {
                            LimbPose (Core::Limb::LEFT_FOOT,  d2r(0), d2r(0), d2r(0), 0, 0, 0)
                        }, "comp_stairs_shift_frontright");


    // Fighting Poses
    // R, P, Y, X, Y, Z
    fight_stance = pose_default +
            RoboPose( std::vector<LimbPose> {
                            LimbPose(Core::Limb::LEFT_HAND,  d2r(20), d2r(0), d2r(0), 0, 0, 0),
                            LimbPose(Core::Limb::RIGHT_HAND, d2r(20), d2r(0), d2r(0), 0, 0, 0),
                            LimbPose(Core::Limb::LEFT_FOOT,  d2r(3), 0, 0, -0.02, -0.02, 0.01),
                            LimbPose(Core::Limb::RIGHT_FOOT, d2r(-1), 0, 0,  0.02,  0.03, 0.025)
                        }, "fight_stance");

    fight_punch_right_forward = fight_stance +
            RoboPose( std::vector<LimbPose> {
                            LimbPose(Core::Limb::LEFT_HAND,  d2r(0), d2r(0), d2r(0), 0, 0, 0),
                            LimbPose(Core::Limb::RIGHT_HAND, d2r(80), d2r(-10), d2r(0), 0, 0, 0)
                        }, "punch_right_forward");

    fight_punch_left_forward = fight_stance +
            RoboPose( std::vector<LimbPose> {
                            LimbPose(Core::Limb::LEFT_HAND,  d2r(80), d2r(10), d2r(0), 0, 0, 0),
                            LimbPose(Core::Limb::RIGHT_HAND, d2r(0), d2r(0), d2r(0), 0, 0, 0)
                        }, "punch_left_forward");

    fight_punch_right_sideways = fight_stance +
            RoboPose( std::vector<LimbPose> {
                            LimbPose(Core::Limb::LEFT_HAND,  d2r(0), d2r(0), d2r(0), 0, 0, 0),
                            LimbPose(Core::Limb::RIGHT_HAND, d2r(0), d2r(-80), d2r(0), 0, 0, 0)
                        }, "punch_right");

    fight_punch_left_sideways = fight_stance +
            RoboPose( std::vector<LimbPose> {
                            LimbPose(Core::Limb::LEFT_HAND,  d2r(0), d2r(80), d2r(0), 0, 0, 0),
                            LimbPose(Core::Limb::RIGHT_HAND, d2r(0), d2r(0), d2r(0), 0, 0, 0)
                        }, "punch_left");


    // Absolute poses:

    // STAIRS absolute
    stairs_shift_toleft = pose_default_stairs + shift_toleft + arms_fwd_right_foot;
    stairs_lift_right = stairs_shift_toleft + lift_right + comp_stairs_shift_toleft;
    stairs_fwd_right = stairs_lift_right + fwd_right + arm_right_fwd + comp_stairs_fwd_right;
    stairs_dual_right = stairs_fwd_right + dual_right - shift_toleft - arms_fwd_right_foot - comp_stairs_shift_toleft - comp_stairs_fwd_right;
    stairs_right_down = stairs_dual_right + comp_stairs_right_down + shift_toright + comp_stairs_shift_toright + arm_left_fwd - arm_right_fwd;
    stairs_shift_frontright = stairs_right_down + lift_left ;
    stairs_lean_frontright = stairs_shift_frontright + lean_fwd_right + fwd_left + comp_stairs_left_pad;
    stairs_adjust_left_pad = pose_default_stairs ;
    stairs_jippie = pose_default_stairs + arms_fwd_left_foot + arms_fwd_left_foot + arms_fwd_right_foot + arms_fwd_right_foot;

    stairs_shift_toleft.objname="stairs_shift_toleft";
    stairs_lift_right.objname="stairs_lift_right";
    stairs_fwd_right.objname="stairs_fwd_right";
    stairs_dual_right.objname="stairs_dual_right";
    stairs_right_down.objname="stairs_right_down";
    stairs_shift_frontright.objname="stairs_shift_frontright";
    stairs_lean_frontright.objname="stairs_lean_frontright";
    stairs_adjust_left_pad.objname="stairs_adjust_left_pad";
    stairs_fwd_left.objname="stairs_fwd_left";

//    stairs_dual_right.printInfo();
//    stairs_right_down.printInfo();

    // walking poses
    // Init
    init_shift_toleft = pose_default+shift_toleft+arms_fwd_right_foot;
    init_lift_right = init_shift_toleft+lift_right+turn_right;
    init_fwd_right = init_lift_right+fwd_right;
    init_dual_right = init_fwd_right+ dual_right- shift_toleft - lift_right - arms_fwd_right_foot;
    init_shift_frontright = init_dual_right+ shift_toright + dual_right-turn_right+arms_fwd_left_foot;

    // Loop
    loop_lift_left = init_shift_frontright+lift_left+turn_left;
    loop_fwd_left = loop_lift_left+fwd_left+fwd_left;
    loop_dual_left = loop_fwd_left+ dual_left- shift_toright - lift_left - arms_fwd_left_foot;
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

double Poses::get_step_length()
{
    return step_length;
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
