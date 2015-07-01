#include <ros/ros.h>

#include "fsm/fsm_defines.h"

#include "core/core.h"
#include "fsm/walk.h"
#include "fsm/fight.h"
#include "kinect.h"
#include "fsm/stairs.h"

#include "poses/poses.h"
#include "poses/parser.h"
#include "core/helper.h"

#define USE_COMPLIANCE 1

int main(int argc, char **argv)
{
    ros::init(argc, argv, "holy");

    std::cout << "HOly package path: " << Helper::getPackagePath("holy") << std::endl;

    Helper::setWorkingDirectory(Helper::getPackagePath("holy"));

    // reading files in the directory set before is working
    //Parser::printCSVRows(Parser::filename);



    Core core(argc, argv);

    Walk walk(&core);
    Stairs stairs(&core);
    Kinect kinect(&core);
    Fight fight(&core);

    Holy_FSM holy_fsm = Holy_FSM::START;
    Holy_FSM holy_fsm_tmp = Holy_FSM::START;

    ros::Rate rate(100);

    ROS_INFO("Waiting for action");

    while(ros::ok()) {
        // STAND
        if (core.get_isstanding()==true) {
            if (core.get_buttons()[static_cast<int>(Controller_Button::L1)] == 1
                    && holy_fsm != Holy_FSM::STAIRS) {
                // GOTO STAIRS
                holy_fsm_tmp = holy_fsm;
                holy_fsm = Holy_FSM::STAIRS;

#if USE_COMPLIANCE
                // trying to call the service for compliance here
                Helper::runComplianceBash(Helper::setBashFilename);
#endif

                // initializing stairs fsm
                stairs.init_StateMachine();
                ROS_INFO("Holy_FSM -> STAIRS");
            }
            else if (core.get_buttons()[static_cast<int>(Controller_Button::R1)]==1
                     && holy_fsm != Holy_FSM::WALK) {
                // GOTO WALK
                holy_fsm_tmp = holy_fsm;
                holy_fsm = Holy_FSM::WALK;

#if USE_COMPLIANCE
                // trying to call the service for compliance here
                if (holy_fsm_tmp == Holy_FSM::STAIRS) {
                    Helper::runComplianceBash(Helper::resetBashFilename);
                }
#endif

                // initializing walk fsm
                walk.init_StateMachine();
                ROS_INFO("Holy_FSM -> WALK");
            }
            else if (core.get_buttons()[static_cast<int>(Controller_Button::L2)]==1
                     && holy_fsm != Holy_FSM::KINECT) {
                // GOTO KINECT
                holy_fsm_tmp = holy_fsm;
                holy_fsm = Holy_FSM::KINECT;

#if USE_COMPLIANCE
                // trying to call the service for compliance here
                if (holy_fsm_tmp == Holy_FSM::STAIRS) {
                    Helper::runComplianceBash(Helper::resetBashFilename);
                }
#endif

                // initializing kinect fsm
                kinect.init_StateMachine();
                ROS_INFO("Holy_FSM -> KINECT");
            }
            else if (core.get_buttons()[static_cast<int>(Controller_Button::R2)]==1
                     && holy_fsm != Holy_FSM::FIGHT) {
                // GOTO FIGHT
                holy_fsm_tmp = holy_fsm;
                holy_fsm = Holy_FSM::FIGHT;

#if USE_COMPLIANCE
                // trying to call the service for compliance here
                if (holy_fsm_tmp == Holy_FSM::STAIRS) {
                    Helper::runComplianceBash(Helper::resetBashFilename);
                }
#endif

                // initializing fight fsm
                fight.init_StateMachine();
                ROS_INFO("Holy_FSM -> FIGHT");
            }
            else if (core.get_buttons()[static_cast<int>(Controller_Button::PS)]==1
                     && holy_fsm != Holy_FSM::START
                     && holy_fsm != Holy_FSM::WAIT) {
                // GOTO FIGHT
                holy_fsm_tmp = holy_fsm;
                holy_fsm = Holy_FSM::START;

#if USE_COMPLIANCE
                // trying to call the service for compliance here
                if (holy_fsm_tmp == Holy_FSM::STAIRS) {
                    Helper::runComplianceBash(Helper::resetBashFilename);
                }
#endif

                ROS_INFO("Holy_FSM -> RELAX");
            }
        }

        // WALK
        if (holy_fsm == Holy_FSM::WALK) {
            walk.StateMachine();
        }
        // STAIRS
        else if (holy_fsm == Holy_FSM::STAIRS) {
            stairs.StateMachine();
        }
        // KINECT
        else if (holy_fsm == Holy_FSM::KINECT) {
            kinect.StateMachine();
        }
        // FIGHT
        else if (holy_fsm == Holy_FSM::FIGHT) {
            fight.StateMachine();
        }
        else if(holy_fsm == Holy_FSM::START) {
            if(core.get_goal_success()) { // initialized with true
                // default position
                core.setPoseTarget(Poses::pose_relax).move(core.get_vel());
                core.set_isstanding(true); // init is true but we should check this
                ROS_INFO("Went to relaxed stance");
                holy_fsm_tmp = holy_fsm;
                holy_fsm = Holy_FSM::WAIT;
            }
        }
        else {
            if (holy_fsm != holy_fsm_tmp) {
                holy_fsm_tmp = holy_fsm;
                ROS_INFO("Idle");
            }
        }

        rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
