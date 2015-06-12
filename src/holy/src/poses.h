/*  HOly Walk walk.cpp
 *  A number of poses that the robot can move to.
 *  Laurenz & Simon 2015-06-11
 */

#ifndef _POSES_H_
#define _POSES_H_

#include <string>
#include <vector>

#include "core.h"

class LimbPose {
public:
    // The standard way of constructing a LimbPose is to say what limb it refers to and state its orientation and position.
    LimbPose(const Core::Limb limb, const double roll, const double pitch, const double yaw, const double x, const double y, const double z)
    : limb(limb),  roll(roll), pitch(pitch), yaw(yaw), x(x), y(y), z(z) {}

    // Use the Core to construct a LimbPose that resembles the actual current Pose of a specific limb
    static LimbPose fromCurrentPose(const Core::Limb limb, Core &core);

    // Adding / Subtracting two LimbPoses merges their values if they refer to the same Core::Limb, but throws a runtime_exception if they have different Core::Limbs.
    LimbPose operator+ (const LimbPose& rlp) const;
    LimbPose operator+=(const LimbPose& rlp);
    LimbPose operator- (const LimbPose& rlp) const;
    LimbPose operator-=(const LimbPose& rlp);

    // Provides a way to cast a LimbPose to an object of the ROS-compatible message type geometry_msgs::Pose
    operator geometry_msgs::Pose () const;

    // The limb this LimbPose is referring to
    Core::Limb limb;

    // The pose of the limb
    double roll, pitch, yaw, x, y, z;

};


class RoboPose {
public:
    // A RoboPose can be constructed from a vector of LimbPoses. Not that the order of the limbs in the vector doesn't matter
    RoboPose(const std::vector<LimbPose> limbs);

    // Use the Core to construct a RoboPose that resembles the actual current Robot Pose
    static RoboPose fromCurrentPose(Core &core);

    // Set a Limb of this RoboPose. Returns a reference to itself for convenient method chaining.
    RoboPose& setLimb(LimbPose limbPose);

    // Get a pointer to a specific limb in this RoboPose. May throw a runtime_exception if the limb is not found
    LimbPose & getLimb(Core::Limb limb);

    // Get all the limbs of this RoboPose
    std::vector<LimbPose>& getLimbs();
    const std::vector<LimbPose> getLimbs() const;

    // Prints orientation and position of contained limbs
    const RoboPose &printInfo() const;
    RoboPose& printInfo();

    // Adding / Subtracting two RoboPoses merges their Limbs: Limbs previously not included in one side will be present in the result, Limbs existing in both RoboPoses will be added / subtracted.
    RoboPose operator+ (const RoboPose& rrp) const;
    RoboPose operator+=(const RoboPose& rrp);
    RoboPose operator- (const RoboPose& rrp) const;
    RoboPose operator-=(const RoboPose& rrp);

private:
    // The limb poses that constitute this RoboPose
    std::vector<LimbPose> limbs;

};


inline double d2r(const double degree)
{
    return M_PI*degree/180.0;
}

inline double r2d(const double radian)
{
    return 180.0*radian/M_PI;
}

// The Poses namespace contains a number of poses that can be used in a state machine with the goal of having the robot fulfill its tasks.
namespace Poses {

// The default pose is the basic pose that must be used as offset for other relatively defined poses
static const LimbPose pose_default_limb_lh (Core::Limb::LEFT_HAND, 1.57142,  -0.0102078,  1.51022,  -0.149827, 0.00525059, 0.0300537);
static const LimbPose pose_default_limb_rh (Core::Limb::RIGHT_HAND,1.57036,  -0.00509431,  -1.48466,  0.14977, 0.00606686, 0.029969);
static const LimbPose pose_default_limb_lf (Core::Limb::LEFT_FOOT, 0.01534,  0.00511267,  0.010305,  -0.0300094, 0.000920314, -0.194994);
static const LimbPose pose_default_limb_rf (Core::Limb::RIGHT_FOOT, -0.00511327,  0.0102263,  0.0152352,  0.0315845, 4.98769e-05, -0.19499);
static const RoboPose pose_default ( std::vector<LimbPose> {
                                         pose_default_limb_lh,
                                         pose_default_limb_rh,
                                         pose_default_limb_lf,
                                         pose_default_limb_rf,
                                     } );

// Rechten Fuss nach aussen ziehen, Insgesamt nach Links lehnen durch pitch auf beiden fuessen
static const RoboPose pose_shift_weight_toleft = pose_default + RoboPose( std::vector<LimbPose> {
                        LimbPose (Core::Limb::RIGHT_FOOT, d2r(0), d2r(8), d2r(0), 0.06, 0, 0),
                        LimbPose (Core::Limb::LEFT_FOOT,  d2r(0), d2r(8), d2r(0), 0, 0, 0),
                                              });

// Rechten Fuss anheben und beide fuesse noch weiter pitchen um servo-verbiegung zu kompensieren
static const RoboPose pose_lift_right_foot = pose_shift_weight_toleft + RoboPose( std::vector<LimbPose> {
                        LimbPose (Core::Limb::RIGHT_FOOT, d2r(0), d2r(4), d2r(0), 0, 0, 0.05),
                        LimbPose (Core::Limb::LEFT_FOOT,  d2r(0), d2r(4), d2r(0), -0.025, -0.008, 0),
                                           });



}

#endif
