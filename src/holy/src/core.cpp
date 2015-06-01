
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

    // asychronous spinner
    aSpin = new ros::AsyncSpinner(0);
    aSpin->start();

    //moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    group = new moveit::planning_interface::MoveGroup("All");
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

std::string Core::getLimbString(Core::Limb limb) const
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
    //    moveGroupAll.move();
}

void Core::setPoseTarget(Core::Limb limb, geometry_msgs::Pose pose)
{
    //    moveGroupAll.setPoseTarget(pose, getLimbString(limb));
}

void Core::moveto_default_state()
{
    std::vector<double> group_variable_values;
    group_variable_values.resize(18);
    //group.getCurrentState()->copyJointGroupPositions(group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()), group_variable_values);
    moveit::planning_interface::MoveGroup::Plan my_plan;
    std::fill(group_variable_values.begin(), group_variable_values.end(), 0.0);
    group->setJointValueTarget(group_variable_values);
    bool success = group->move();
    //group->execute(my_plan);
    ROS_INFO("Visualizing plan 2 (joint space goal) %s",success?"":"FAILED");
}
