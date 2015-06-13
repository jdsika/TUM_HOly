#include "poses.h"

#include <string>
#include <vector>
#include <map>
#include "limbpose.h"
#include "../core.h"

const RoboPose Poses::pose_default(std::vector<LimbPose> {
                                        LimbPose(Core::Limb::LEFT_HAND,  d2r(90),  0, d2r( 90), -0.15, 0, 0.03),
                                        LimbPose(Core::Limb::RIGHT_HAND, d2r(90),  0, d2r(-90),  0.15, 0, 0.03),
                                        LimbPose(Core::Limb::LEFT_FOOT,  0,  0,  0, -0.03, 0, -0.2),
                                        LimbPose(Core::Limb::RIGHT_FOOT, 0,  0,  0,  0.03, 0, -0.2)
                                    } , "pose_default");

const RoboPose Poses::pose_shift_weight_toleft = Poses::pose_default + RoboPose( std::vector<LimbPose> {
                                                                                      LimbPose (Core::Limb::RIGHT_FOOT, d2r(0), d2r(8), d2r(0), 0.06, 0, 0),
                                                                                      LimbPose (Core::Limb::LEFT_FOOT,  d2r(0), d2r(8), d2r(0), 0, 0, 0),
                                                                                  }, "pose_shift_weight_toleft");

const RoboPose Poses::pose_lift_right_foot = Poses::pose_shift_weight_toleft + RoboPose( std::vector<LimbPose> {
                                                                                             LimbPose (Core::Limb::RIGHT_FOOT, d2r(0), d2r(4), d2r(0), 0, 0, 0.05),
                                                                                             LimbPose (Core::Limb::LEFT_FOOT,  d2r(0), d2r(4), d2r(0), -0.025, -0.008, 0),
                                                                                         }, "pose_lift_right_foot");


Poses::Poses()
{

}

Poses::~Poses()
{

}
