/*  HOly Walk walk.cpp
 *  A number of poses that the robot can move to.
 *  Laurenz & Simon 2015-06-11
 */

#ifndef _POSES_H_
#define _POSES_H_


#include <map>
#include <string>
#include <vector>

#include "core.h"

struct pose {
    double roll, pitch, yaw, x, y, z;
    bool relative; // Meaning of values is either relative or absolute
    const geometry_msgs::Pose toGeoPose() const;
    void print(bool progformat = false) const;
};

// default pose is considered as offset for relative pose
static const std::map<Core::Limb, pose> pose_default {
    {Core::Limb::LEFT_HAND, pose {1.57142,  -0.0102078,  1.51022,  -0.149827, 0.00525059, 0.0300537} },
    {Core::Limb::RIGHT_HAND, pose {1.57036,  -0.00509431,  -1.48466,  0.14977, 0.00606686, 0.029969} },
    {Core::Limb::LEFT_FOOT, pose {0.01534,  0.00511267,  0.010305,  -0.0300094, 0.000920314, -0.194994} },
    {Core::Limb::RIGHT_FOOT, pose {-0.00511327,  0.0102263,  0.0152352,  0.0315845, 4.98769e-05, -0.19499} }
};

static std::map<Core::Limb, pose > pose_gewicht_links {
    {Core::Limb::LEFT_FOOT, pose {0, 0, 0, 0, 0, 0} },
    {Core::Limb::RIGHT_FOOT, pose {0, 0, 0, 0, 0, 0} }
};



#endif
