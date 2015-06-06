
#include "core.h"
#include <ros/spinner.h>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_listener.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

//#include <moveit_msgs/AttachedCollisionObject.h>
//#include <moveit_msgs/CollisionObject.h>

Core::Core(int argc, char** argv)

{
    // Initialize ROS System
    ros::init(argc, argv, "holy_walk");
    node_handle = new ros::NodeHandle;

    // asychronous spinner creating as many threads as there are cores
    aSpin = new ros::AsyncSpinner(0);
    aSpin->start();

    //moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    group = new moveit::planning_interface::MoveGroup("All");
    //moveit::planning_interface::MoveGroup group_left_arm = moveit::planning_interface::MoveGroup("LeftArm");
    //moveit::planning_interface::MoveGroup group_right_arm = moveit::planning_interface::MoveGroup("RightArm");
    //group->setGoalOrientationTolerance(2*M_PI);
    //group->setGoalPositionTolerance(0.01);



    ros::Publisher display_publisher = node_handle->advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    moveit_msgs::DisplayTrajectory display_trajectory;

    // Wait for TF to appear
    listener = new tf::TransformListener;
    transforms[Limb::RIGHT_FOOT] = new tf::StampedTransform; // Right Foot
    transforms[Limb::LEFT_FOOT] = new tf::StampedTransform; // etc
    transforms[Limb::RIGHT_HAND] = new tf::StampedTransform;
    transforms[Limb::LEFT_HAND] = new tf::StampedTransform;

    bool tryagain = true;
    ROS_INFO("Waiting for TF to appear");
    while(tryagain)
    {
        try{
            tryagain = false;
            listener->lookupTransform("/base_link", "/R_foot_pad", ros::Time(0), *transforms[Limb::RIGHT_FOOT]);
            listener->lookupTransform("/base_link", "/L_foot_pad", ros::Time(0), *transforms[Limb::LEFT_FOOT]);
            listener->lookupTransform("/base_link", "/R_forearm", ros::Time(0), *transforms[Limb::RIGHT_HAND]);
            listener->lookupTransform("/base_link", "/L_forearm", ros::Time(0), *transforms[Limb::LEFT_HAND]);
        }
        catch (tf::TransformException ex){
            //std::cout << ex.what() << std::endl;
            std::cout << "." ;
            std::flush(std::cout);
            ros::Duration(1.0).sleep();
            tryagain = true;
        }
    }


}

Core::~Core()
{
    ros::shutdown();

    for(auto tf : transforms){
        delete tf.second;
    }
    delete listener;
    delete group;
    delete node_handle;
    delete aSpin;
}

tf::StampedTransform *Core::getTF(Core::Limb limb)
{
    return transforms[limb];
}

moveit::planning_interface::MoveGroup &Core::getMoveGroup()
{
    return *group;
}

const std::string Core::getLimbString(Core::Limb limb)
{
    std::string limb_str;
    switch(limb)
    {
    case Limb::LEFT_FOOT:
        limb_str = "L_foot_pad";
        break;
    case Limb::RIGHT_FOOT:
        limb_str = "R_foot_pad";
        break;
    case Limb::LEFT_HAND:
        limb_str = "L_forearm";
        break;
    case Limb::RIGHT_HAND:
        limb_str = "R_forearm";
        break;
    default:
        std::cerr << "unexpected limb enum " << static_cast<int>(limb) << std::endl;
        break;
    }
    return limb_str;
}

void Core::move()
{
       group->move();
}

void Core::setPoseTarget(Core::Limb limb, geometry_msgs::Pose pose)
{
    // this will ensure the movement of arms (same as checkbox in rvis)
    moveit::core::RobotStatePtr robot_state=group->getCurrentState();
    // enable Approximate IK solutions
    kinematics::KinematicsQueryOptions kQO;
    kQO.return_approximate_solution=true;
    //kQO.lock_redundant_joints=true;
//    robot_state->printStatePositions();
    bool success=robot_state->setFromIK(robot_state->getJointModelGroup("RightArm"), // Group
                           pose, // pose
                           3, // Attempts
                           1.0, // timeout
                           moveit::core::GroupStateValidityCallbackFn(), // Contraint
                           kQO); // enable Approx IK
    std::cout<< success << std::endl;
//    std::cout << *robot_state->getJointPositions("L_HAA");
    std::vector<double> positions;
//    robot_state->printStatePositions();
    robot_state->copyJointGroupPositions(robot_state->getJointModelGroup("All"),positions);
    group->setJointValueTarget(positions);
   // group->setPoseTarget(pose, getLimbString(limb));
}


void Core::moveto_default_state()
{
    std::vector<double> group_variable_values;
    // all 18 joints are set to 0.0 position
    group_variable_values.resize(18);
    //group.getCurrentState()->copyJointGroupPositions(group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()), group_variable_values);
    moveit::planning_interface::MoveGroup::Plan my_plan;
    std::fill(group_variable_values.begin(), group_variable_values.end(), 0.0);

    // assign values to group
    group->setJointValueTarget(group_variable_values);

    // this will plan and execute in one step
    bool success = group->move();

    ROS_INFO("Visualizing plan 2 (joint space goal) %s",success?"":"FAILED");
}
