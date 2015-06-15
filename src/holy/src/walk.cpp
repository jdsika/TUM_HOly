#include "walk.h"

#include "core.h"
#include "poses/poses.h"


Walk::Walk(Core *core) : core{core}
{

}

Walk::~Walk()
{

}

void Walk::executeStateMachine()
{
    // parse files before each walking attempt
    if(!Poses::parseRoboPositions(Poses::filename)) return;

    // all loaded walking poses will be executed
    // do not start wih position_default
    for(int i = 0; i < Poses::walkingPoses.size();++i)
    {
        if(!ros::ok()) return;
        core->setPoseTarget(Poses::walkingPoses.at(i)).move();
    }

    if(!ros::ok()) return;
    ros::spinOnce();
}


