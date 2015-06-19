#ifndef PARSER_H
#define PARSER_H

#include "robopose.h"

#include <string>
#include <vector>

class Parser
{
public:
    Parser();

    static bool printCSVRows(std::string filename);
    static bool parseRoboPositions(std::string filename);
    static bool getWorkingDirectory();

    static const std::string filename;
    static std::vector<RoboPose> walkingPoses;

private:
};

#endif // PARSER_H
