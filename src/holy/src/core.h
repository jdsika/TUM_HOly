/*  HOly Walk Core.cpp
 *  Talk to MoveIt Planner and talk to Bioloid motors
 *  Laurenz & Simon 2015-05-29
 */

#ifndef _CORE_H_
#define _CORE_H_

#include <map>

#include <moveit/robot_model/robot_model.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_interface/planning_interface.h>
#include <sensor_msgs/Joy.h>
#include "actionlib_msgs/GoalStatusArray.h"

// Forward declarations
namespace ros{
class NodeHandle;
class AsyncSpinner;
}
namespace tf{
class TransformListener;
class StampedTransform;
}

class LimbPose;
class RoboPose;

class Core {
public:
    Core(int argc, char **argv);
    ~Core();

    enum class Limb {ERROR, LEFT_HAND, RIGHT_HAND, LEFT_FOOT, RIGHT_FOOT};

    tf::StampedTransform* getTF(Limb limb);

    moveit::planning_interface::MoveGroup & getMoveGroup();

    Core& setPoseTarget(const RoboPose &rp);
    Core& setPoseTarget(const LimbPose& lp);

    // static methods to identify Limbs and Groups by enum
    static const std::string getLimbString(const Core::Limb limb);
    static const std::string getLimbGroup (const Core::Limb limb);
    static const Core::Limb getLimbEnum(const std::string limbString);

    Core& move(const double speed_scale = 1.0);

    Core& moveto_default_state();

    // Control inputs getter/setter
    bool get_stop();

    double get_vel();

    double get_turning_angle();

    bool set_stop(bool yes_no);

    bool set_vel(double vel);

    double set_turning_angle(double angle);

    bool get_goal_success();

    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

    void goalCallback(const actionlib_msgs::GoalStatusArrayConstPtr& goal);

private:
    // Control inputs
    bool stop;
    double velocity,turning_angle;

    std::string controller;

    ros::NodeHandle *node_handle;

    moveit::planning_interface::MoveGroup *group;

    tf::TransformListener *listener;
    std::map<Limb, tf::StampedTransform*> transforms;

    ros::AsyncSpinner *aSpin;

    moveit::core::RobotStatePtr robot_state;

    bool goal_success;

    ros::Subscriber goal_sub;

    ros::Subscriber joy_sub;

    void updateTF();
};

#endif
