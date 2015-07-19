#include "parser.h"

#include "limbpose.h"
#include "robopose.h"

#include "poses.h"

#include "../core/core.h"


#include "../CSV/csviterator.h"


const std::string Parser::filename = "positions.csv";


Parser::Parser()
{
}

std::vector<RoboPose> Parser::walkingPoses = std::vector<RoboPose> ();

Poses poses;

bool Parser::printCSVRows(std::string filename)
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

bool Parser::parseRoboPositions(std::string filename)
{
    Parser::walkingPoses.resize(0);

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
            Parser::walkingPoses.resize(0);
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
                    Parser::walkingPoses.push_back(poses.pose_default);
                    continue;
                }
                else {
                    Parser::walkingPoses.push_back(Parser::walkingPoses.at(numberOfPositions-1));
                }
            }
        }
        // get LimbString
        Core::Limb limb = Core::getLimbEnum((*loop)[0]);

        if(limb == Core::Limb::ERROR) {
            Parser::walkingPoses.resize(0);
            std::cout << "PARSE FAILED -> wrong limb name" << std::endl;
            return false;
        }

        std::vector<LimbPose> limbPosVector;
        limbPosVector.resize(1);

        limbPosVector[0] = LimbPose(limb,
                                    Poses::d2r(::atof(((*loop)[1]).c_str())),
                                    Poses::d2r(::atof(((*loop)[2]).c_str())),
                                    Poses::d2r(::atof(((*loop)[3]).c_str())),
                                        ::atof(((*loop)[4].c_str())),
                                        ::atof(((*loop)[5].c_str())),
                                        ::atof(((*loop)[6]).c_str()));

        Parser::walkingPoses[numberOfPositions] += RoboPose(limbPosVector,
                                                    currentPosName);

        Parser::walkingPoses[numberOfPositions].setRoboPosName(currentPosName);
    }

    return true;
}
