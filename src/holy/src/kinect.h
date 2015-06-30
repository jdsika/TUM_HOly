/*  HOly KINECT KINECT.cpp
 *  Ways to move limbs that make the robot KINECT
 *  Laurenz & Simon 2015-05-29
 */

#ifndef _KINECT_H_
#define _KINECT_H_

#include "poses/poses.h"
#include "fsm/fsm_defines.h"
#include "poses/robopose.h"
#include <tf/transform_listener.h>


class Core;

class Kinect {
public:
    Kinect(Core* core);
    ~Kinect();

    Kinect_FSM kinect_fsm;

    // State machine
    void StateMachine();

    void init_StateMachine();
    tf::TransformListener* kin_listener;

    std::map<Core::Limb, tf::StampedTransform> kin_transforms;
    double compute_norm(tf::StampedTransform &frame1);

    RoboPose pose_from_kinect;
    RoboPose pose_dual_stand;
    RoboPose pose_right_stand;
    RoboPose pose_left_stand;
    void updateTF();
    void human_to_robopose();


private:
    Core* core;
    // poses object
    Poses kinect_poses;

};

#endif
