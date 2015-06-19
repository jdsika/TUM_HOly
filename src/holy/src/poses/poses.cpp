#include "poses.h"

#include "limbpose.h"
#include "../core.h"

#include "../CSV/csviterator.h"
#include <stdio.h>  /* defines FILENAME_MAX */
#ifdef WINDOWS
    #include <direct.h>
    #define GetCurrentDir _getcwd
#else
    #include <unistd.h>
    #define GetCurrentDir getcwd
 #endif

//
//const std::string Poses::filename = "positions.csv";
const std::string Poses::filename = "position_test.csv";

// R, P, Y, X, Y, Z
// Feet: Toes up(+) down(-) / ankles left(+) right(-) / twist left right / X / Y / Z
// Hands: Arms up down / ? / ? / (X) / (Y) / (Z)

// Invert for L/R:
// Feet:  No / Yes / Yes / Yes / No / No
// Hands: No / ?   / Yes / Yes / No / No

RoboPose Poses::pose_default(std::vector<LimbPose> {
                       //changed to have lower arms
                                       LimbPose(Core::Limb::LEFT_HAND,  d2r(90),  0, d2r(90-1), -0.15, 0, 0.03),
                                       LimbPose(Core::Limb::RIGHT_HAND, d2r(90),  0, d2r(-90+1),  0.15, 0, 0.03),
                                       LimbPose(Core::Limb::LEFT_FOOT,  d2r(2),  0,  0, -0.03, 0.0, -0.2),
                                       LimbPose(Core::Limb::RIGHT_FOOT, d2r(2),  0,  0,  0.03, 0.0, -0.2)
                                   } , "pose_default");



// General relative poses, easy to change them all at once, do not call them seperate
RoboPose Poses::shift_toleft =
        RoboPose( std::vector<LimbPose> {
                        LimbPose(Core::Limb::LEFT_HAND,  d2r(30),  0, 0, 0, 0, 0),
                        LimbPose(Core::Limb::RIGHT_HAND, d2r(-70),  0, 0,  0, 0, 0),
                        LimbPose (Core::Limb::LEFT_FOOT,  d2r(0), d2r(14), d2r(0), 0, 0, 0),
                        LimbPose (Core::Limb::RIGHT_FOOT, d2r(0), d2r(14), d2r(0), 0, 0, Poses::compute_Z_off(14)*-0.5),
                    }, "shift_toleft");

RoboPose Poses::shift_toright =
        RoboPose( std::vector<LimbPose> {
                        LimbPose(Core::Limb::LEFT_HAND,  d2r(-70),  0, 0, 0, 0, 0),
                        LimbPose(Core::Limb::RIGHT_HAND, d2r(30),  0, 0,  0, 0, 0),
                        LimbPose (Core::Limb::LEFT_FOOT,  d2r(0), d2r(-13), d2r(0), 0, 0, Poses::compute_Z_off(13)*-0.5),
                        LimbPose (Core::Limb::RIGHT_FOOT, d2r(0), d2r(-13), d2r(0), 0, 0, 0),
                    }, "shift_toright");

RoboPose Poses::lift_right =
        RoboPose( std::vector<LimbPose> {
                        LimbPose (Core::Limb::RIGHT_FOOT,  d2r(0), d2r(0), d2r(0), 0.02, 0,0.02), // Compensate x due to weak motors
                    }, "lift_right");

RoboPose Poses::lift_left =
        RoboPose( std::vector<LimbPose> {
                        LimbPose (Core::Limb::LEFT_FOOT,  d2r(0), d2r(0), d2r(0), -0.02, 0,0.02),
                    }, "lift_left");

RoboPose Poses::fwd_left =
        RoboPose( std::vector<LimbPose> {
                        LimbPose (Core::Limb::LEFT_FOOT,  d2r(0), d2r(0), d2r(0), 0, 0.02,0),
                        LimbPose (Core::Limb::RIGHT_FOOT,  d2r(0), d2r(0), d2r(0), 0, 0,0),
                    }, "fwd_left");

RoboPose Poses::fwd_right =
        RoboPose( std::vector<LimbPose> {
                        LimbPose (Core::Limb::RIGHT_FOOT,  d2r(0), d2r(0), d2r(0), 0, 0.02,0),
                        LimbPose (Core::Limb::LEFT_FOOT,  d2r(0), d2r(0), d2r(0), 0, 0,0),
                    }, "fwd_left");

RoboPose Poses::dual_right =
        RoboPose( std::vector<LimbPose> {
                      LimbPose (Core::Limb::LEFT_FOOT,  d2r(0), d2r(0), d2r(0), 0, -Poses::fwd_right.getLimb(Core::Limb::RIGHT_FOOT).y/2, 0),
                      LimbPose (Core::Limb::RIGHT_FOOT, d2r(0), d2r(0), d2r(0), 0, -Poses::fwd_right.getLimb(Core::Limb::RIGHT_FOOT).y/2, 0),
                  }, "dual_right") ;

RoboPose Poses::dual_left =
        RoboPose( std::vector<LimbPose> {
                      LimbPose (Core::Limb::LEFT_FOOT,  d2r(0), d2r(0), d2r(0), 0, -Poses::fwd_right.getLimb(Core::Limb::LEFT_FOOT).y/2, 0),
                      LimbPose (Core::Limb::RIGHT_FOOT, d2r(0), d2r(0), d2r(0), 0, -Poses::fwd_right.getLimb(Core::Limb::LEFT_FOOT).y/2, 0),
                  }, "dual_left") ;

// Absolute poses:
// Init
RoboPose Poses::init_shift_toleft = Poses::pose_default+Poses::shift_toleft;
RoboPose Poses::init_shift_toright = Poses::pose_default+Poses::shift_toright;
RoboPose Poses::init_lift_left = Poses::init_shift_toright+Poses::lift_left;
RoboPose Poses::init_lift_right = Poses::init_shift_toleft+Poses::lift_right;
RoboPose Poses::init_fwd_left = Poses::init_lift_left+Poses::fwd_left;
RoboPose Poses::init_fwd_right = Poses::init_lift_right+Poses::fwd_right;
RoboPose Poses::init_dual_right = Poses::init_fwd_right+ Poses::dual_right- Poses::shift_toleft - Poses::lift_right;
RoboPose Poses::init_shift_frontright = Poses::init_dual_right+ Poses::shift_toright + Poses::dual_right ;

// Loop
RoboPose Poses::loop_lift_left = Poses::init_shift_frontright+Poses::lift_left;
RoboPose Poses::loop_fwd_left = Poses::loop_lift_left+Poses::fwd_left+Poses::fwd_left;
RoboPose Poses::loop_dual_left = Poses::loop_fwd_left+ Poses::dual_left- Poses::shift_toright - Poses::lift_left;
RoboPose Poses::loop_shift_frontleft = Poses::loop_dual_left+ Poses::shift_toleft + Poses::dual_left ;
RoboPose Poses::loop_lift_right = Poses::loop_shift_frontleft+Poses::lift_right;
RoboPose Poses::loop_fwd_right = Poses::loop_lift_right+Poses::fwd_right+Poses::fwd_right;
RoboPose Poses::loop_dual_right = Poses::loop_fwd_right+ Poses::dual_right- Poses::shift_toleft - Poses::lift_right;
RoboPose Poses::loop_shift_frontright = Poses::loop_dual_right + Poses::shift_toright + Poses::dual_right;

// Stop
RoboPose Poses::stop_lift_left = Poses::init_shift_frontright+Poses::lift_left;
RoboPose Poses::stop_fwd_left = Poses::stop_lift_left+Poses::fwd_left;

/* Old poses
const RoboPose Poses::pose_shift_weight_toright = Poses::pose_default
        + RoboPose( std::vector<LimbPose> {
                        LimbPose (Core::Limb::LEFT_FOOT,  d2r(0), d2r(-10), d2r(0), -0.04, 0, 0),
                        LimbPose (Core::Limb::RIGHT_FOOT, d2r(0), d2r(-10), d2r(0), -0.02, 0, 0),
                    }, "pose_shift_weight_toright");

const RoboPose Poses::pose_lift_left_foot = Poses::pose_shift_weight_toright
        + RoboPose( std::vector<LimbPose> {
                        LimbPose (Core::Limb::RIGHT_HAND, d2r(60), d2r(0), d2r(0), 0, 0, 0),
                        LimbPose (Core::Limb::LEFT_FOOT,  d2r(-5), d2r(10), d2r(0), 0.02, 0, 0.03),
                        LimbPose (Core::Limb::RIGHT_FOOT, d2r(0), d2r(-5), d2r(0), 0.02, -0.0075, 0),
                    }, "pose_lift_left_foot");

const RoboPose Poses::pose_left_foot_advance_forward = Poses::pose_lift_left_foot
        + RoboPose( std::vector<LimbPose> {
                        LimbPose (Core::Limb::LEFT_FOOT,  d2r(0), d2r(0), d2r(0), 0, 0.03, 0),
                    }, "pose_left_foot_advance_forward");

const RoboPose Poses::pose_left_foot_advanced_down =
        RoboPose( std::vector<LimbPose> {
                      Poses::pose_default.getLimb(Core::Limb::LEFT_HAND),
                      Poses::pose_default.getLimb(Core::Limb::RIGHT_HAND),
                      Poses::pose_left_foot_advance_forward.getLimb(Core::Limb::RIGHT_FOOT),
                      Poses::pose_left_foot_advance_forward.getLimb(Core::Limb::LEFT_FOOT)
                      + LimbPose (Core::Limb::LEFT_FOOT,  d2r(5), d2r(-10), d2r(0), 0, 0, -0.03),
                    }, "pose_left_foot_advanced_down");

const RoboPose Poses::pose_left_foot_advanced_shiftweighttoleft =
        RoboPose( std::vector<LimbPose> {
                      Poses::pose_default.getLimb(Core::Limb::LEFT_HAND),
                      Poses::pose_default.getLimb(Core::Limb::RIGHT_HAND),
                      Poses::pose_left_foot_advanced_down.getLimb(Core::Limb::LEFT_FOOT)
                      + LimbPose (Core::Limb::LEFT_FOOT,  d2r(0), d2r(+10), d2r(0), 0.06, -0.015, 0),
                      Poses::pose_left_foot_advanced_down.getLimb(Core::Limb::RIGHT_FOOT)
                      + LimbPose (Core::Limb::RIGHT_FOOT, d2r(0), d2r(+10+5), d2r(0), 0.04, -0.015, 0),
                    }, "pose_left_foot_advanced_shiftweighttoleft");


const RoboPose Poses::pose_shift_weight_toleft = Poses::pose_default
        + RoboPose( std::vector<LimbPose> {
                        LimbPose (Core::Limb::RIGHT_FOOT,  d2r(0), d2r(14), d2r(0), 0.04, 0, 0),
                        LimbPose (Core::Limb::LEFT_FOOT, d2r(0), d2r(14), d2r(0), 0.02, 0, 0),
                    }, "pose_shift_weight_toleft");

const RoboPose Poses::pose_lift_right_foot = Poses::pose_shift_weight_toleft
        + RoboPose( std::vector<LimbPose> {
                        LimbPose (Core::Limb::LEFT_HAND, d2r(60), d2r(0), d2r(0), 0, 0, 0),
                        LimbPose (Core::Limb::RIGHT_FOOT,  d2r(-5), d2r(-14), d2r(0), -0.02, 0, 0.03),
                        LimbPose (Core::Limb::LEFT_FOOT, d2r(0), d2r(5), d2r(0), -0.02, -0.0075, 0),
                    }, "pose_lift_right_foot");

const RoboPose Poses::pose_right_foot_advance_forward = Poses::pose_lift_right_foot
        + RoboPose( std::vector<LimbPose> {
                        LimbPose (Core::Limb::RIGHT_FOOT,  d2r(0), d2r(0), d2r(0), 0, 0.03, 0),
                    }, "pose_right_foot_advance_forward");

const RoboPose Poses::pose_right_foot_advanced_down =
        RoboPose( std::vector<LimbPose> {
                      Poses::pose_default.getLimb(Core::Limb::LEFT_HAND),
                      Poses::pose_default.getLimb(Core::Limb::RIGHT_HAND),
                      Poses::pose_right_foot_advance_forward.getLimb(Core::Limb::LEFT_FOOT),
                      Poses::pose_right_foot_advance_forward.getLimb(Core::Limb::RIGHT_FOOT)
                      + LimbPose (Core::Limb::RIGHT_FOOT,  d2r(5), d2r(+10), d2r(0), 0, 0, -0.03),
                    }, "pose_right_foot_advanced_down");

const RoboPose Poses::pose_right_foot_advanced_shiftweighttoright =
        RoboPose( std::vector<LimbPose> {
                      Poses::pose_default.getLimb(Core::Limb::LEFT_HAND),
                      Poses::pose_default.getLimb(Core::Limb::RIGHT_HAND),
                      Poses::pose_right_foot_advanced_down.getLimb(Core::Limb::RIGHT_FOOT)
                      + LimbPose (Core::Limb::RIGHT_FOOT,  d2r(0), d2r(-10), d2r(0), -0.06, -0.015, 0),
                      Poses::pose_right_foot_advanced_down.getLimb(Core::Limb::LEFT_FOOT)
                      + LimbPose (Core::Limb::LEFT_FOOT, d2r(0), d2r(-14-5), d2r(0), -0.04, -0.015, 0),
                    }, "pose_right_foot_advanced_shiftweighttoright");

*/



double Poses::compute_Z_off(double angle)
{
    return tan(angle* M_PI / 180.0)*(0.078); // von der Mitte der Gelenke gemessen
}

std::vector<RoboPose> Poses::walkingPoses = std::vector<RoboPose> ();


Poses::Poses()
{

}

Poses::~Poses()
{

}

bool Poses::printCSVRows(std::string filename)
{

    int i = 1;
    std::ifstream file(filename, std::ifstream::in);

    for(CSV::Iterator loop(file); loop != CSV::Iterator(); ++loop)
    {
        std::cout << "Rows Nr.: " << i << std::endl;

        for(int j=0; j < (*loop).size(); j++)
        {
            std::cout << "Element: " << j << " / " << (*loop)[j] << std::endl;
        }
         i++;
    }
}

bool Poses::parseRoboPositions(std::string filename)
{
    Poses::walkingPoses.resize(0);

    std::string currentPosName;
    std::string priorPosName = "init";
    int numberOfPositions = -1;

    std::ifstream file(filename, std::ifstream::in);

    //iterator through rows
    CSV::Iterator loop(file);

    // skip first row
    ++loop;

    for(loop; loop != CSV::Iterator(); ++loop)
    {
        // correct table needs 8 values
        if ((*loop).size() == 7) {
            continue;
        }
        else if ((*loop).size() != 8) {
            Poses::walkingPoses.resize(0);
            std::cout << "PARSE FAILED -> wrong size: " << (*loop).size() << std::endl;
            return false;
        }

        // get position name
        currentPosName = (*loop)[7];

        if(currentPosName == "finished")
            break;

        // check if position changed
        if(currentPosName != priorPosName) {
            if (currentPosName == "continue") {
                ++loop;
                currentPosName = (*loop)[7];
                priorPosName = currentPosName;
            }
            else {
                numberOfPositions++;
                priorPosName = currentPosName;
                if (currentPosName == "pose_default") {
                    Poses::walkingPoses.push_back(Poses::pose_default);
                    continue;
                }
                else {
                    Poses::walkingPoses.push_back(Poses::walkingPoses.at(numberOfPositions-1));
                }
            }
        }
        // get LimbString
        Core::Limb limb = Core::getLimbEnum((*loop)[0]);

        if(limb == Core::Limb::ERROR) {
            Poses::walkingPoses.resize(0);
            std::cout << "PARSE FAILED -> wrong limb name" << std::endl;
            return false;
        }

        std::vector<LimbPose> limbPosVector;
        limbPosVector.resize(1);

        limbPosVector[0] = LimbPose(limb,
                                    d2r(::atof(((*loop)[1]).c_str())),
                                    d2r(::atof(((*loop)[2]).c_str())),
                                    d2r(::atof(((*loop)[3]).c_str())),
                                        ::atof(((*loop)[4].c_str())),
                                        ::atof(((*loop)[5].c_str())),
                                        ::atof(((*loop)[6]).c_str()));

        Poses::walkingPoses[numberOfPositions] += RoboPose(limbPosVector,
                                                    currentPosName);

        Poses::walkingPoses[numberOfPositions].objname = currentPosName;
    }

    return true;
}

bool Poses::getWorkingDirectory()
{
    char cCurrentPath[FILENAME_MAX];

     if (!GetCurrentDir(cCurrentPath, sizeof(cCurrentPath)))
         {
         return false;
         }

    cCurrentPath[sizeof(cCurrentPath) - 1] = '\0'; /* not really required */

    std::cout << "The current working directory is: " << cCurrentPath << std::endl;

    std::string path = std::string(cCurrentPath);

    path = path.substr(0, path.size()-4);

    chdir(path.c_str());
}
