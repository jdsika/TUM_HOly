
#include "core.h"
#include <ros/spinner.h>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_listener.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

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
//    group->setGoalOrientationTolerance(5.0*M_PI/180.0);
    group->setGoalTolerance(0.001);
    //group->setGoalJointTolerance(5*M_PI/180.0);



    ros::Publisher display_publisher = node_handle->advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    moveit_msgs::DisplayTrajectory display_trajectory;

    // Wait for TF to appear
    listener = new tf::TransformListener;
    transforms[Limb::RIGHT_FOOT] = new tf::StampedTransform; // Right Foot
    transforms[Limb::LEFT_FOOT] = new tf::StampedTransform; // etc
    transforms[Limb::RIGHT_HAND] = new tf::StampedTransform;
    transforms[Limb::LEFT_HAND] = new tf::StampedTransform;

    ROS_INFO("Waiting for TF to appear");
    listener->waitForTransform("/base_link", "/R_foot_pad",ros::Time(0), ros::Duration(5));
    updateTF();

    robot_state = group->getCurrentState();

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
    updateTF();
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

const std::string Core::getLimbGroup(Core::Limb limb)
{
    std::string limb_str;
    switch(limb)
    {
    case Limb::LEFT_FOOT:
        limb_str = "LeftLeg";
        break;
    case Limb::RIGHT_FOOT:
        limb_str = "RightLeg";
        break;
    case Limb::LEFT_HAND:
        limb_str = "LeftArm";
        break;
    case Limb::RIGHT_HAND:
        limb_str = "RightArm";
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

    std::cout << "Pose Orient:\n  " << pose.orientation.x << " / " << pose.orientation.y << " / " << pose.orientation.z << " / " << pose.orientation.w << "\n  "
              << pose.position.x << "m / " << pose.position.y << "m / " << pose.position.z  << "m\n";

    tf::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
    double r,p,y;
    tf::Matrix3x3(tf::Quaternion(pose.orientation.x,
                                 pose.orientation.y,
                                 pose.orientation.z,
                                 pose.orientation.w)
                  ).getEulerYPR(y, p, r);
//    std::cout << "Pose Orient converted to R/P/Y:\n  " << r*180.0/M_PI << "° / " << p*180.0/M_PI << "° / " << y*180.0/M_PI << "°" << std::endl;



    // this will ensure the movement of arms (same as checkbox in rvis)
    // enable Approximate IK solutions for the hands only
    kinematics::KinematicsQueryOptions kQO;
    kQO.return_approximate_solution = (limb == Core::Limb::LEFT_HAND || limb == Core::Limb::RIGHT_HAND)? true : false;

    std::cout << "Before" << std::endl;
//    robot_state->printStatePositions();

    robot_state->update(true);

    bool success=robot_state->setFromIK(robot_state->getJointModelGroup(Core::getLimbGroup(limb)), // Group
                                        pose, // pose
                                        333, // Attempts
                                        1.0, // timeout
                                        moveit::core::GroupStateValidityCallbackFn(), // Contraint
                                        kQO); // enable Approx IK

    std::cout<< "After ("<<(success?"OK":"Failed")<<")" << std::endl;
//    robot_state->printStatePositions();
    //    std::cout << *robot_state->getJointPositions("L_HAA");

    std::vector<double> positions;
    positions.resize(18);
    std::fill(positions.begin(), positions.end(), 0.0);
    robot_state->copyJointGroupPositions(robot_state->getJointModelGroup("All"),positions);
//    std::cout << "Positions: ";
//    for (int i=0; i<positions.size(); ++i)
//    {
//        if(fabs(positions[i]) >= 0.999* M_PI) {positions[i]=0.0; std::cout << " changed: "; }
//        std::cout << positions[i]*180.0/M_PI << "°, ";
//    }
//    std::cout << std::endl;
    group->setJointValueTarget(positions);
}


void Core::moveto_default_state()
{
    std::vector<double> group_variable_values;
    // all 18 joints are set to 0.0 position
    group_variable_values.resize(18);
    std::fill(group_variable_values.begin(), group_variable_values.end(), 0.0);
//    group_variable_values[3] = -0.6; // bend knees
//    group_variable_values[3+9] = 0.6;

    for(std::string s : group->getCurrentState()->getJointModelGroup("All")->getJointModelNames())
    {
        std::cout << s << ": " << *(group->getCurrentState()->getJointPositions(s)) << ", ";
    }
    // L_HAA, L_HR, L_HFE, L_KFE, L_AFE, L_AR, L_SAA, L_SFE, L_EB, R_HAA, R_HR, R_HFE, R_KFE, R_AFE, R_AR, R_SAA, R_SFE, R_EB

    // assign values to group
    group->setJointValueTarget(group_variable_values);

    // this will plan and execute in one step
    bool success = group->move();

    ROS_INFO("Visualizing plan 2 (joint space goal) %s",success?"":"FAILED");
}

void Core::updateTF()
{
    listener->lookupTransform("/base_link", "/R_foot_pad", ros::Time(0), *transforms[Limb::RIGHT_FOOT]);
    listener->lookupTransform("/base_link", "/L_foot_pad", ros::Time(0), *transforms[Limb::LEFT_FOOT]);
    listener->lookupTransform("/base_link", "/R_forearm", ros::Time(0), *transforms[Limb::RIGHT_HAND]);
    listener->lookupTransform("/base_link", "/L_forearm", ros::Time(0), *transforms[Limb::LEFT_HAND]);
}
