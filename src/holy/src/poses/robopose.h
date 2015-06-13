/*  HOly Walk walk.cpp
 *  A number of poses that the robot can move to.
 *  Laurenz & Simon & Carlo 2015-06-11
 */
#ifndef ROBOPOSE_H
#define ROBOPOSE_H

#include <string>
#include <vector>

#include "../core.h"


class LimbPose;

class RoboPose {
public:
    // default init
    RoboPose();
    // A RoboPose can be constructed from a vector of LimbPoses. Not that the order of the limbs in the vector doesn't matter
    RoboPose(const std::vector<LimbPose> limbs, const std::string objname = "");

    // Use the Core to construct a RoboPose that resembles the actual current Robot Pose
    static RoboPose fromCurrentPose(Core &core);

    // Set a Limb of this RoboPose. Returns a reference to itself for convenient method chaining.
    RoboPose& setLimb(LimbPose limbPose);

    // Get a pointer to a specific limb in this RoboPose. May throw a runtime_exception if the limb is not found
    LimbPose & getLimb(Core::Limb limb);

    // Get all the limbs of this RoboPose
    std::vector<LimbPose>& getLimbs();
    const std::vector<LimbPose> getLimbs() const;

    // return the name of the position
    const std::string getRoboPosName() const;

    // Prints orientation and position of contained limbs
    const RoboPose &printInfo() const;
    RoboPose& printInfo();

    // Adding / Subtracting two RoboPoses merges their Limbs: Limbs previously not included in one side will be present in the result, Limbs existing in both RoboPoses will be added / subtracted.
    RoboPose operator+ (const RoboPose& rrp) const;
    RoboPose& operator+=(const RoboPose& rrp);
    RoboPose operator- (const RoboPose& rrp) const;
    RoboPose& operator-=(const RoboPose& rrp);

private:
    // The limb poses that constitute this RoboPose
    std::vector<LimbPose> limbs;

    std::string objname;

    void printInfoImpl() const;

};
#endif // ROBOPOSE_H
