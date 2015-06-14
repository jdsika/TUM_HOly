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
const std::string Poses::filename = "positions.csv";

// R, P, Y, X, Y, Z
// Feet: Toes up(+) down(-) / ankles left(+) right(-) / twist left right / X / Y / Z
// Hands: Arms up down / ? / ? / (X) / (Y) / (Z)

// Invert for L/R:
// Feet:  No / Yes / Yes / Yes / No / No
// Hands: No / ?   / Yes / Yes / No / No

const RoboPose Poses::pose_default(std::vector<LimbPose> {
                       //changed to have lower arms
                                       LimbPose(Core::Limb::LEFT_HAND,  d2r(90-70),  0, d2r(90-1), -0.15, 0, 0.03),
                                       LimbPose(Core::Limb::RIGHT_HAND, d2r(90-70),  0, d2r(-90+1),  0.15, 0, 0.03),
                                       LimbPose(Core::Limb::LEFT_FOOT,  0,  0,  0, -0.03, 0.005, -0.2+0.01),
                                       LimbPose(Core::Limb::RIGHT_FOOT, 0,  0,  0,  0.03, 0.005, -0.2+0.01)
                                   } , "pose_default");






const RoboPose Poses::pose_shift_weight_toright = Poses::pose_default
        + RoboPose( std::vector<LimbPose> {
                        LimbPose (Core::Limb::LEFT_FOOT,  d2r(0), d2r(-10), d2r(0), -0.04, 0, 0),
                        LimbPose (Core::Limb::RIGHT_FOOT, d2r(0), d2r(-10), d2r(0), -0.02, 0, 0),
                    }, "pose_shift_weight_toleft");

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
        if ((*loop).size() != 8) {
            Poses::walkingPoses.resize(0);
            std::cout << "PARSE FAILED" << std::endl;
            return false;
        }

        // get position name
        currentPosName = (*loop)[7];

        // check if position changed
        if(currentPosName != priorPosName) {
            numberOfPositions++;
            priorPosName = currentPosName;
            if (numberOfPositions == 0)
                Poses::walkingPoses.push_back(Poses::pose_default);
            else
                Poses::walkingPoses.push_back(Poses::walkingPoses.at(numberOfPositions-1));
        }
        // get LimbString
        Core::Limb limb = Core::getLimbEnum((*loop)[0]);

        if(limb == Core::Limb::ERROR) {
            Poses::walkingPoses.resize(0);
            std::cout << "PARSE FAILED" << std::endl;
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
}
