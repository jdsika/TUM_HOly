
#include "core.h"

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <geometry_msgs/Pose.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <tf/transform_listener.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_interface/planning_interface.h>
#include <pluginlib/class_loader.h>

Core::Core(int argc, char** argv) //:
//    moveGroupAll("All")
//    planning_scene(new planning_scene::PlanningScene(robot_model))
{

    // Initialize ROS System
    ros::init(argc, argv, "holy_walk");
    node_handle = new ros::NodeHandle;
    ros::Publisher display_publisher = node_handle->advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);

    // Moveit
    robot_model_loader::RobotModelLoader rml("robot_description");
    robot_model = rml.getModel();

    planning_scene = planning_scene::PlanningScenePtr(new planning_scene::PlanningScene(robot_model));

    boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager> > planner_plugin_loader;
    std::string planner_plugin_name;
    if (!node_handle->getParam("planning_plugin", planner_plugin_name))
        ROS_FATAL_STREAM("Could not find planner plugin name");
    try
    {
        planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>("moveit_core", "planning_interface::PlannerManager"));
    }
    catch(pluginlib::PluginlibException& ex)
    {
        ROS_FATAL_STREAM("Exception while creating planning plugin loader " << ex.what());
    }
    try
    {
        planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
        if (!planner_instance->initialize(robot_model, node_handle->getNamespace()))
            ROS_FATAL_STREAM("Could not initialize planner instance");
        ROS_INFO_STREAM("Using planning interface '" << planner_instance->getDescription() << "'");
    }
    catch(pluginlib::PluginlibException& ex)
    {
        const std::vector<std::string> &classes = planner_plugin_loader->getDeclaredClasses();
        std::stringstream ss;
        for (std::size_t i = 0 ; i < classes.size() ; ++i)
            ss << classes[i] << " ";
        ROS_ERROR_STREAM("Exception while loading planner '" << planner_plugin_name << "': " << ex.what() << std::endl
                         << "Available plugins: " << ss.str());
    }

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
            std::cout << ex.what() << std::endl;
            std::cout << "." ;
            std::flush(std::cout);
            ros::Duration(1.0).sleep();
            tryagain = true;
        }
    }


}

Core::~Core()
{
    delete node_handle;
    delete listener;
    for(auto tf : transforms){
        delete tf.second;
    }
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
    robot_state::RobotState goal_state(robot_model);
    const robot_state::JointModelGroup* jmg = robot_model->getJointModelGroup("All");
    req.group_name = "All";
    std::cout << jmg->getJointModelNames()[0] << std::endl;
    std::vector<double> joint_values(18, 0.0); // all 18 joints at 0deg
    goal_state.setJointGroupPositions(jmg, joint_values);
    moveit_msgs::Constraints joint_goal = kinematic_constraints::constructGoalConstraints(goal_state, jmg);
    req.goal_constraints.clear();
    req.goal_constraints.push_back(joint_goal);
    context = planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
    context->solve(res);
    if(res.error_code_.val != res.error_code_.SUCCESS)
    {
        ROS_ERROR("Could not compute plan successfully");
    }
}
