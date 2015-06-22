#include "walk.h"

#include "core.h"
#include "poses/poses.h"
#include "poses/parser.h"


Walk::Walk(Core *core) : core{core}
{

    walk_fsm=Walk::STAND;
    init_fsm=Walk::iSHIFT_LEFT;
    loop_fsm=Walk::lFWD_LEFT;
    stop_fsm=Walk::sFWD_LEFT;
}

Walk::~Walk()
{

}

void Walk::executeStateMachine()
{
    // parse files before each walking attempt
    if(!Parser::parseRoboPositions(Parser::filename)) return;

    // all loaded walking poses will be executed
    // do not start wih position_default
    for(int i = 0; i < Parser::walkingPoses.size();++i)
    {
        if(!ros::ok()) return;
        core->setPoseTarget(Parser::walkingPoses.at(i)).move();
    }

    if(!ros::ok()) return;
    ros::spinOnce();
}


