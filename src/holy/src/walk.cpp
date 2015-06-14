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

    ros::Rate rate(0.25);

    // all loaded walking poses will be executed
    for(int i = 0; i < Poses::walkingPoses.size();++i)
    {
        core->setPoseTarget(Poses::walkingPoses.at(i)).move();

        if(!ros::ok()) break;
        ros::spinOnce();
        rate.sleep();
    }  
}


