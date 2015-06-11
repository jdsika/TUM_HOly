
#include "core.h"

#include <map>

#include <ros/spinner.h>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_listener.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/robot_model.h>
#include "poses.h"

std::map<std::string, double> map_min, map_max;

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
    group->setGoalOrientationTolerance(0.0000050*M_PI/180.0);
    group->setGoalPositionTolerance(0.0000002);
    group->setGoalJointTolerance(0.00005*M_PI/180.0);



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

    // Bring limbs to zero position
    moveto_default_state();

    // Jetzt Robot State abspeichern, damit Start State von späteren Planungen nicht der Power-on State ist
    //robot_state = group->getCurrentState();
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kin_model = robot_model_loader.getModel();
    robot_state = moveit::core::RobotStatePtr (new moveit::core::RobotState(kin_model));
    robot_state->setToDefaultValues();


    std::vector<std::string> IDs {"L_HAA", "L_HR", "L_HFE", "L_KFE", "L_AFE", "L_AR", "L_SAA", "L_SFE", "L_EB", "R_HAA", "R_HR", "R_HFE", "R_KFE", "R_AFE", "R_AR", "R_SAA", "R_SFE", "R_EB"};
    for(std::string id : IDs)
    {
        double val, init;
        ros::param::get(id+"_controller/motor/init", init);
        ros::param::get(id+"_controller/motor/min", val);
        map_min[id] = (val - init + 512) * 2.0*M_PI / 1024.0 - M_PI;
        ros::param::get(id+"_controller/motor/max", val);
        map_max[id] = (val - init + 512) * 2.0*M_PI / 1024.0 - M_PI;
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

bool groupStateValidityCallback(
        moveit::core::RobotState *robot_state,
        const moveit::core::JointModelGroup *joint_group,
        const double *joint_group_variable_values
        )
{
    for(int i=0; i<joint_group->getJointModelNames().size(); ++i)
    {
        const std::string id = joint_group->getJointModelNames().at(i);
        const double val = joint_group_variable_values[i];
        if( val < map_min.at(id) )
        {
            std::cout << "rejecting "<<joint_group->getJointModelNames().at(i)<<": " << joint_group_variable_values[i]<<", because < " << map_min.at(id)<<std::endl;
            return false;
        }
        if( val > map_max.at(id))
        {
            std::cout << "rejecting "<<joint_group->getJointModelNames().at(i)<<": " << joint_group_variable_values[i]<<", because > " << map_max.at(id)<<std::endl;
            return false;
        }
    }

    return true;
}

void Core::setPoseTarget(Core::Limb limb, geometry_msgs::Pose pose)
{
    // this will ensure the movement of arms (same as checkbox in rvis)
    // enable Approximate IK solutions for the hands only
    kinematics::KinematicsQueryOptions kQO;
    kQO.return_approximate_solution = false; //(limb == Core::Limb::LEFT_HAND || limb == Core::Limb::RIGHT_HAND)? true : false;

//    bool success = group->setPoseTarget(pose, getLimbString(limb));


    //std::cout << "---\n";
//    robot_state->printStatePositions();

    bool success=robot_state->setFromIK(robot_state->getJointModelGroup(Core::getLimbGroup(limb)), // Group
                                        pose, // pose
                                        30, // Attempts
                                        2.0, // timeout
                                        groupStateValidityCallback, // Constraint
                                        kQO); // enable Approx IK
    if(!success)
    {
        std::cout<< "setFromIK Failed" << std::endl;
    }

    std::vector<double> positions;
    robot_state->copyJointGroupPositions("All",positions);

//    std::cout << ">>>\n";
    //robot_state->printStatePositions();


    group->setJointValueTarget(positions);
}


void Core::moveto_default_state()
{
    std::vector<double> group_variable_values;
    // all 18 joints are set to 0.0 position
    group_variable_values.resize(18);
    std::fill(group_variable_values.begin(), group_variable_values.end(), 0.0);
    // Order: L_HAA, L_HR, L_HFE, L_KFE, L_AFE, L_AR, L_SAA, L_SFE, L_EB, R_HAA, R_HR, R_HFE, R_KFE, R_AFE, R_AR, R_SAA, R_SFE, R_EB

    // assign values to group
    bool success = group->setJointValueTarget(group_variable_values);
    if(!success) std::cout << "setJointValueTarget() failed" << std::endl;

    // this will plan and execute in one step
    success = group->move();
    if(!success) std::cout << "moveto_default_state() failed" << std::endl;
}

void Core::updateTF()
{
    listener->lookupTransform("/base_link", "/R_foot_pad", ros::Time(0), *transforms[Limb::RIGHT_FOOT]);
    listener->lookupTransform("/base_link", "/L_foot_pad", ros::Time(0), *transforms[Limb::LEFT_FOOT]);
    listener->lookupTransform("/base_link", "/R_forearm", ros::Time(0), *transforms[Limb::RIGHT_HAND]);
    listener->lookupTransform("/base_link", "/L_forearm", ros::Time(0), *transforms[Limb::LEFT_HAND]);
}

pose Core::getCurrentPose(Core::Limb limb)
{

    geometry_msgs::Pose currPos;
    tf::StampedTransform* transf = getTF(limb);
    tf::Matrix3x3 matOrient;
    matOrient.setRotation(transf->getRotation());
    tf::Quaternion quat;
    matOrient.getRotation(quat);
    currPos.orientation.x = quat.getX();
    currPos.orientation.y = quat.getY();
    currPos.orientation.z = quat.getZ();
    currPos.orientation.w = quat.getW();

    // translation addieren
    currPos.position.x = static_cast<double>(transf->getOrigin().x());
    currPos.position.y = static_cast<double>(transf->getOrigin().y());
    currPos.position.z = static_cast<double>(transf->getOrigin().z());

    double r,p,y;
    tf::Matrix3x3(tf::Quaternion(currPos.orientation.x,
                                 currPos.orientation.y,
                                 currPos.orientation.z,
                                 currPos.orientation.w)
                  ).getEulerYPR(y, p, r);

    struct pose currentPose = {
        r, p, y,
        currPos.position.x,
        currPos.position.y,
        currPos.position.z
    };
    return currentPose;
}
