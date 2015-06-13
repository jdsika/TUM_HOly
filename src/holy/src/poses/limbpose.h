/*  HOly Walk walk.cpp
 *  A number of poses that the robot can move to.
 *  Laurenz & Simon & Carlo 2015-06-11
 */
#ifndef LIMBPOSE_H
#define LIMBPOSE_H

#include "../core.h"

class LimbPose {
public:
    LimbPose(): limb(Core::Limb::ERROR),  roll(0.0), pitch(0.0), yaw(0.0), x(0.0), y(0.0), z(0.0) {}
    // The standard way of constructing a LimbPose is to say what limb it refers to and state its orientation and position.
    LimbPose(const Core::Limb limb,
             const double roll,
             const double pitch,
             const double yaw,
             const double x,
             const double y,
             const double z) : limb(limb),  roll(roll), pitch(pitch), yaw(yaw), x(x), y(y), z(z) {}

    // Use the Core to construct a LimbPose that resembles the actual current Pose of a specific limb
    static LimbPose fromCurrentPose(const Core::Limb limb, Core &core);

    // Adding / Subtracting two LimbPoses merges their values if they refer to the same Core::Limb, but throws a runtime_exception if they have different Core::Limbs.
    LimbPose operator+ (const LimbPose& rlp) const;
    LimbPose& operator+=(const LimbPose& rlp);
    LimbPose operator- (const LimbPose& rlp) const;
    LimbPose& operator-=(const LimbPose& rlp);

    // Provides a way to cast a LimbPose to an object of the ROS-compatible message type geometry_msgs::Pose
    operator geometry_msgs::Pose () const;

    // The limb this LimbPose is referring to
    Core::Limb limb;

    // The pose of the limb
    double roll;
    double pitch;
    double yaw;
    double x;
    double y;
    double z;

};

#endif // LIMBPOSE_H
