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
    static bool parseRoboPoses(std::string filename);

    static bool getWorkingDirectory();

public:
    // The default pose is the basic pose that must be used as offset for other relatively defined poses
    static const RoboPose pose_default;

    // Rechten Fuss nach aussen ziehen, Insgesamt nach Links lehnen durch pitch auf beiden fuessen
    static const RoboPose pose_shift_weight_toleft;

    // Rechten Fuss anheben und beide fuesse noch weiter pitchen um servo-verbiegung zu kompensieren
    static const RoboPose pose_lift_right_foot;

    static std::vector<RoboPose> walkingPoses;
};

#endif
