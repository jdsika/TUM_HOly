#include <ros/ros.h>

#include "holy_fsm_defines.h"

#include "core.h"
#include "walk.h"
#include "fight.h"
#include "kinect.h"
#include "stairs.h"

#include "poses/poses.h"

//#include <ros/service.h>    // this is used to set the compliance slope, margin and punch
//#include <string>
//#include <vector>

//bool setCompliance()
//{
//    std::vector<std::string> side = {"/L_","/R_"};
//    std::vector<std::string> joint = {"HAA", "HR", "HFE", "KFE", "AFE", "AR"};
//    std::vector<std::string> service = {"_controller/set_speed",
//                           "_controller/torque_enable",
//                          "_controller/set_compliance_slope",
//                          "_controller/set_compliance_margin",
//                          "_controller/set_compliance_punch"
//                          };

//    std::vector<bool> answers;

//    answers.resize(side.size()*joint.size()*service.size());

//    std::vector<double> values = {5.3, 1.0, 12.0, 0.0, 0.0};

//    for( std::string s : side) {
//        for( std::string m : joint) {
//            for(int i = 0; i < values.size(); ++i)
//            {
//                //ros::service::call(s+m+service.at(i), values[i], answers[i]);
//                //std::cout << s+m+service.at(i) << std::endl;
//            }
//        }
//    }


//    return true;
//}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "holy_walk");

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

                // trying to call the service for compliance here
                //setCompliance();

                // initializing stairs fsm
                stairs.init_StateMachine();
                ROS_INFO("Holy_FSM -> STAIRS");
            }
            else if (core.get_buttons()[static_cast<int>(Controller_Button::R1)]==1
                     && holy_fsm != Holy_FSM::WALK) {
                // GOTO WALK
                holy_fsm_tmp = holy_fsm;
                holy_fsm = Holy_FSM::WALK;

                // initializing walk fsm
                walk.init_StateMachine();
                ROS_INFO("Holy_FSM -> WALK");
            }
            else if (core.get_buttons()[static_cast<int>(Controller_Button::L2)]==1
                     && holy_fsm != Holy_FSM::KINECT) {
                // GOTO KINECT
                holy_fsm_tmp = holy_fsm;
                holy_fsm = Holy_FSM::KINECT;

                // initializing kinect fsm
                kinect.init_StateMachine();
                ROS_INFO("Holy_FSM -> KINECT");
            }
            else if (core.get_buttons()[static_cast<int>(Controller_Button::R2)]==1
                     && holy_fsm != Holy_FSM::FIGHT) {
                // GOTO FIGHT
                holy_fsm_tmp = holy_fsm;
                holy_fsm = Holy_FSM::FIGHT;

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
