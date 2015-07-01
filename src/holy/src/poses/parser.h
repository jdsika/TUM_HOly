#ifndef PARSER_H
#define PARSER_H

#include "robopose.h"

#include <string>
#include <vector>

class Parser
{
private:
    Parser();

public:
    static bool printCSVRows(std::string filename);
    static bool parseRoboPositions(std::string filename);

    static const std::string filename;
    static std::vector<RoboPose> walkingPoses;
};

#endif // PARSER_H
