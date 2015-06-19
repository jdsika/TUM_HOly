#include <cmath>

#include <vector>
#include <string>
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <moveit/move_group_interface/move_group.h>
#include <geometry_msgs/Pose.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <tf/transform_listener.h>

#include "core.h"
#include "walk.h"
#include "poses/poses.h"
#include "poses/parser.h"
#include "poses/robopose.h"


int main(int argc, char **argv)
{
    Parser::getWorkingDirectory();
    Core core(argc, argv);
    Walk walk(&core);


    // In Start Position gehen
    core.setPoseTarget(Poses::pose_default).move();

    ros::Duration(0.5).sleep();

    //
    // LimbPose parameterization test
    //

    LimbPose lp = Poses::pose_default.getLimb(Core::Limb::RIGHT_HAND);

    // fuege neuen parameter Roll-Influence hinzu, der zum standard roll-wert 10* den input wert hinzufuegt
    lp.setParameterAdd("Roll-Influence", 10, 0, 0 , 0, 0, 0);

    // setze den input fuer Roll-Influence auf 2. Roll sollte jetzt standard * 1.0 + 10*2 sein.
    lp.setParameterInput("Roll-Influence", 2.0);

    core.setPoseTarget(lp).move();
    ros::Duration(2.0).sleep();


    // Parameter "entfernen" indem einfluss auf 0 gesetzt wird
    lp.setParameterAdd("Roll-Influence", 0, 0, 0 , 0, 0, 0);

    // Neuen Parameter hinzufuegen der roll wert auf standard * 1.3*input aendert
    lp.setParameterMul("Roll-Influence Multiplikativ", 1.3, 0, 0, 0, 0, 0);

    // input fuer parameter
    lp.setParameterInput("Roll-Influence Multiplikativ", -3.0);

    // Roll sollte standard * 1.3*(-2.0) + 0 sein
    core.setPoseTarget(lp).move();

    return 0;


    //std::cout << "Walk (w) or Stairs (s) ?" << std::endl;
    //std::coud << "Your choice: ";
    //std::cin >> input;
    while(ros::ok()) {

        if (walk.walk_fsm==Walk::STAND) {
            // Stand

            if (!core.get_stop()) {
                walk.walk_fsm=Walk::INIT;
            }
        }

        else if (walk.walk_fsm==Walk::INIT) {

            //Init
            core.setPoseTarget(Poses::init_shift_toleft).move();
            core.setPoseTarget(Poses::init_lift_right).move();
            core.setPoseTarget(Poses::init_fwd_right).move();
            core.setPoseTarget(Poses::init_dual_right).move();
            core.setPoseTarget(Poses::init_shift_frontright).move();

            // Go to Loop
            walk.walk_fsm=Walk::LOOP;
        }

        else if (walk.walk_fsm==Walk::LOOP) {

            // Loop
            core.setPoseTarget(Poses::loop_lift_left).move();
            core.setPoseTarget(Poses::loop_fwd_left).move();
            core.setPoseTarget(Poses::loop_dual_left).move();
            core.setPoseTarget(Poses::loop_shift_frontleft).move();
            core.setPoseTarget(Poses::loop_lift_right).move();
            core.setPoseTarget(Poses::loop_fwd_right).move();
            core.setPoseTarget(Poses::loop_dual_right).move();
            core.setPoseTarget(Poses::loop_shift_frontright).move();

            // Go to stop if control input
            if (core.get_stop()) {
                walk.walk_fsm=Walk::LOOP;
            }
        }
        else if (walk.walk_fsm==Walk::STOP) {

            // Stop
            core.setPoseTarget(Poses::stop_lift_left).move();
            core.setPoseTarget(Poses::stop_fwd_left).move();
            core.setPoseTarget(Poses::pose_default).move();
            walk.walk_fsm=Walk::STAND;
        }
        rate.sleep();
        ros::spinOnce();

    //walk.executeStateMachine();*/

    while(ros::ok())
    {
        // Init
        core.setPoseTarget(Poses::init_shift_toleft).move();
        core.setPoseTarget(Poses::init_lift_right).move();
        core.setPoseTarget(Poses::init_fwd_right).move();
        core.setPoseTarget(Poses::init_dual_right).move();
        core.setPoseTarget(Poses::init_shift_frontright).move();
        // Loop
        core.setPoseTarget(Poses::loop_lift_left).move();
        core.setPoseTarget(Poses::loop_fwd_left).move();
        core.setPoseTarget(Poses::loop_dual_left).move();
        core.setPoseTarget(Poses::loop_shift_frontleft).move();
        core.setPoseTarget(Poses::loop_lift_right).move();
        core.setPoseTarget(Poses::loop_fwd_right).move();
        core.setPoseTarget(Poses::loop_dual_right).move();
        core.setPoseTarget(Poses::loop_shift_frontright).move();
        // Stop
        core.setPoseTarget(Poses::stop_lift_left).move();
        core.setPoseTarget(Poses::stop_fwd_left).move();
        core.setPoseTarget(Poses::pose_default).move();


        //walk.executeStateMachine();

        ros::spinOnce();
>>>>>>> c3918450851da6f9c667406da8c48f0b366edd7f
    }

    return 0;

}
