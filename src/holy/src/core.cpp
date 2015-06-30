
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
#include <moveit/planning_interface/planning_interface.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Joy.h>
#include "poses/limbpose.h"
#include "poses/robopose.h"
#include "actionlib_msgs/GoalStatusArray.h"

#define DEBUG 0

std::map<std::string, double> map_min, map_max;

Core::Core(int argc, char** argv)

{
    // Initialize ROS System
    node_handle = new ros::NodeHandle;

    // asychronous spinner creating as many threads as there are cores
    aSpin = new ros::AsyncSpinner(0);
    aSpin->start();

    group = new moveit::planning_interface::MoveGroup("All");
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
    //moveto_default_state();

    // Jetzt Robot State abspeichern, damit Start State von späteren Planungen nicht der Power-on State ist
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kin_model = robot_model_loader.getModel();
    robot_state = moveit::core::RobotStatePtr (new moveit::core::RobotState(kin_model));
    robot_state->setToDefaultValues();


    std::vector<std::string> IDs {"L_HAA", "L_HR", "L_HFE", "L_KFE", "L_AFE", "L_AR", "L_SAA", "L_SFE", "L_EB", "R_HAA", "R_HR", "R_HFE", "R_KFE", "R_AFE", "R_AR", "R_SAA", "R_SFE", "R_EB"};
    for(std::string id : IDs)
    {
        ros::param::get("/robot_description_planning/joint_limits/"+id+"/max_position", map_max[id]);
        ros::param::get("/robot_description_planning/joint_limits/"+id+"/min_position", map_min[id]);
    }

    // limits arms so we dont point weirdly
    map_max["L_EB"] =  0.1;
    map_min["L_EB"] = -0.1;
    map_max["R_EB"] =  0.1;
    map_min["R_EB"] = -0.1;

    // Subscribe to Joystick messages
    joy_sub = node_handle->subscribe<sensor_msgs::Joy>("joy", 10, &Core::joyCallback, this);

    stop = false; // Change to 1 for default
    isstanding = false; // we have to get in the default position at the beginning
    velocity = 1.0;
    turning_angle = 0;
    step_length = 0;
    temp_velocity=velocity;
    temp_turning=turning_angle;
    temp_step_length=step_length;
    // Subscribe to goal status
    goal_sub = node_handle->subscribe<actionlib_msgs::GoalStatusArray>("/holy_joint_trajectory_action_controller/follow_joint_trajectory/status", 10, &Core::goalCallback, this);

    // we have to initialize the values with a minimum in order to not crash
    buttons = {0,0,0,0,0,0,0,0,0,0,0,0};
    init_buttons = false;
    show_buttons = false;

    this->set_goal_success(true);
    goal_id_of_last_success="";

    ros::param::get("/move_group/moveit_controller_manager",controller);
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
void Core::goalCallback(const actionlib_msgs::GoalStatusArrayConstPtr& goal)
{
    bool r = true;

    if (!goal->status_list.empty())
    {
        for (int i=0; i<goal->status_list.size(); i++)
        {
            if (goal->status_list[i].status!=3)
            {
                r = false; // at least one state is not succeeded, which means it is generally not right yet
                break;
            }
        }
    }

    if(r && !goal->status_list.empty())
    {
        // state says succeeded, but is it the new pose already?
        std::string new_id = goal->status_list.back().goal_id.id;
        if(new_id != static_cast<std::string>(goal_id_of_last_success)) // ID did change, which is what we want
        {
            //ROS_INFO("GOAL SUCCESS == TRUE");
            this->set_goal_success(true);
            goal_id_of_last_success = new_id;
        }
    }
}

double Core::getStep_length() const
{
    return step_length;
}



void Core::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    // it depends on the joystick how many buttons there are on the pad
    if (!init_buttons)
    {
        if(buttons.size() != joy->buttons.size())
        {
            buttons.resize(joy->buttons.size());
            for(int i = 0; i < buttons.size(); ++i)
            {
                buttons[i] = 0;
            }
        }
        init_buttons = true;
    }

    // if the previous button combination has changed the new will be stored
    bool zeros = std::all_of(joy->buttons.begin(), joy->buttons.end(), [](int i) { return i==0; });

    if(!zeros) {
        // always fetch the buttons
        buttons=joy->buttons;

        if (show_buttons && DEBUG) {
            std::cout << "Button values: ";
            for(int values : buttons) {
                std::cout << values << " ";
            }
            std::cout << std::endl;
            show_buttons = false;
        }
    }
    else
    {
        for(int i = 0; i < buttons.size(); ++i)
        {
            buttons[i] = 0;
        }
        show_buttons = true;
    }

    if (joy->axes[1]<0.02){
        stop=true;
    }
    else {
        stop=false;

        velocity = 1.0 + joy->axes[1] * 12; // adjust vel
        turning_angle = 20 * joy->axes[2]; // adjust turn
        step_length = 0.03 * joy->axes[3]; // adjust stepsize

        // Buttons: 0-> , 1-> ,2 ->
        // Display the values if they have changed quite a bit from what was displayed before
        if((fabs(temp_velocity - velocity) > 0.1 || fabs(temp_turning - turning_angle) > 0.1 || fabs(temp_step_length - step_length) > 0.001) && DEBUG)
        {
            ROS_INFO("vel: %.2f, angle: %.2f, step_length: %.2fcm", velocity, turning_angle, 100*step_length);
            temp_turning = turning_angle;
            temp_velocity = velocity;
            temp_step_length = step_length;
        }

        // When walking reverse, turn like you expect it from driving a car
        if(step_length < 0)
        {
            turning_angle *= -1;
        }
    }
}


bool Core::get_goal_success() {
    goal_success_checker_locker.lock();
    bool goal_success_temp = goal_success;
    goal_success_checker_locker.unlock();
    return goal_success_temp;
}

void Core::set_goal_success(const bool yes_no)
{
    goal_success_checker_locker.lock();
    goal_success = yes_no;
    goal_success_checker_locker.unlock();
}

std::vector<int> Core::get_buttons() const{
    return buttons;
}

void Core::set_buttons(const int position, const int value)
{
    if (position < buttons.size())
        buttons[position] = value;
}

bool Core::get_stop() {
    return stop;
}

double Core::get_vel() {
    return velocity;
}

double Core::get_vel_slow() const
{
    return velocity / 3;
}

double Core::get_turning_angle() {
    return turning_angle;
}

bool Core::set_stop(bool yes_no) {
    stop=yes_no;
}

bool Core::set_vel(double vel) {
    velocity=vel;
}

double Core::set_turning_angle(double angle) {
    turning_angle=angle;
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

const std::string Core::getLimbString(const Core::Limb limb)
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

const std::string Core::getLimbGroup(const Core::Limb limb)
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

const Core::Limb Core::getLimbEnum(const std::string limbString)
{
    if(Core::getLimbString(Limb::LEFT_FOOT) == limbString)
        return Limb::LEFT_FOOT;
    else if(Core::getLimbString(Limb::RIGHT_FOOT) == limbString)
        return Limb::RIGHT_FOOT;
    else if(Core::getLimbString(Limb::LEFT_HAND) == limbString)
        return Limb::LEFT_HAND;
    else if(Core::getLimbString(Limb::RIGHT_HAND) == limbString)
        return Limb::RIGHT_HAND;
    else
        return Limb::ERROR;

}

static std::vector<double> last_positions(18);

Core &Core::move(const double speed_scale)
{
    std::vector<double> end_positions(18);
    group->getJointValueTarget().copyJointGroupPositions("All", end_positions);
    if( last_positions != end_positions)
    {
        this->set_goal_success(false);

        last_positions = end_positions;
        //group->move();
        moveit::planning_interface::MoveGroup::Plan plan;
        bool success = group->plan(plan);


        if(success)
        {
            const ros::Duration startTime = plan.trajectory_.joint_trajectory.points.front().time_from_start;
            std::vector<double> start_positions = plan.trajectory_.joint_trajectory.points.front().positions;
            std::vector<double> length(18);
            std::vector<double> acc(18);
            int max_length_id=0;
            double max_length=-1;

            for( int i=0; i<18; i++ )
            {
                length[i]=plan.trajectory_.joint_trajectory.points.back().positions[i]-plan.trajectory_.joint_trajectory.points.front().positions[i];
                if (fabs(length[i])>max_length) {
                    max_length=fabs(length[i]);
                    max_length_id=i;
                }
            }
            double a_max = speed_scale;
            for( int i=0; i<18; i++ )
            {
                acc[i]=a_max*(length[i]/max_length);

            }

            double t_end=2*sqrt(max_length/a_max)+startTime.toSec();
            double num_points = 100;
            plan.trajectory_.joint_trajectory.points.resize(num_points,plan.trajectory_.joint_trajectory.points.front());
            double delta_t = (t_end - startTime.toSec())/(num_points-1);

            for( int point=0; point<num_points; point++ )
            {
                plan.trajectory_.joint_trajectory.points[point].time_from_start = ros::Duration().fromSec(startTime.toSec() + delta_t*point);
                if (point<num_points/2) {
                    for (int joint=0; joint<18; joint++) {
                        plan.trajectory_.joint_trajectory.points[point].positions[joint]=start_positions[joint]+0.5*acc[joint]*(point*delta_t)*(point*delta_t);
                    }

                }
                else {
                    for (int joint=0; joint<18; joint++) {
                        plan.trajectory_.joint_trajectory.points[point].positions[joint]=end_positions[joint]-0.5*acc[joint]*((num_points-1-point)*delta_t)*((num_points-1-point)*delta_t);
                    }
                }
                for (int joint=0; joint<18; joint++) {
                    plan.trajectory_.joint_trajectory.points[point].accelerations[joint] = acc[joint];
                }
                for (int joint=0; joint<18; joint++) {
                    if(point == 0 || point == num_points-1)
                    {
                        plan.trajectory_.joint_trajectory.points[point].velocities[joint]= 0;
                    } else {
                        double p_last = plan.trajectory_.joint_trajectory.points[point-1].positions[joint];
                        double p_next = plan.trajectory_.joint_trajectory.points[point+1].positions[joint];
                        plan.trajectory_.joint_trajectory.points[point].velocities[joint] = (p_next - p_last) / (2 * delta_t);
                    }

                }

            }

            group->asyncExecute(plan);

            // if simulation, wait a second and go on
            if (controller=="moveit_fake_controller_manager/MoveItFakeControllerManager") {
                ros::Duration(0.3).sleep();
                this->set_goal_success(true);
            }

        }
    }
    return *this;
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
//            std::cout << "rejecting "<<joint_group->getJointModelNames().at(i)<<": " << joint_group_variable_values[i]<<", because < " << map_min.at(id)<<std::endl;
            return false;
        }
        if( val > map_max.at(id))
        {
//            std::cout << "rejecting "<<joint_group->getJointModelNames().at(i)<<": " << joint_group_variable_values[i]<<", because > " << map_max.at(id)<<std::endl;
            return false;
        }
    }

    return true;
}

Core &Core::setPoseTarget(const RoboPose &rp)
{
    //std::cout << "Set targets for \""<<rp.objname<<"\"..."<<std::endl;

    for(const LimbPose lp : rp.getLimbs())
    {
        setPoseTarget(lp);
    }

    return *this;
}

Core &Core::setPoseTarget(const LimbPose &lp)
{
    kinematics::KinematicsQueryOptions kQO;
    kQO.return_approximate_solution = false; // not needed with ikfast!

    bool success=robot_state->setFromIK(robot_state->getJointModelGroup(Core::getLimbGroup(lp.limb)), // Group
                                        lp, // pose
                                        1, // Attempts
                                        2.0, // timeout
                                        groupStateValidityCallback, // Constraint
                                        kQO); // enable Approx IK
    if(!success)
    {
        std::cout<< "setFromIK for " << getLimbString(lp.limb) << " failed" << std::endl;
    } else {
        std::vector<double> positions;
        robot_state->copyJointGroupPositions("All",positions);
        group->setJointValueTarget(positions);
    }

    return *this;
}


Core &Core::moveto_default_state()
{
    /*
     * Name - Dxl ID - Meaning
     *
     * R_SAA -  1 - Right Shoulder (rotates)
     * R_SFE -  3 - Right Biceps (bends)
     * R_EB  -  5 - Right Elbow
     * R_HAA -  7 - Right Hip (rotates)
     * R_HR  -  9 - Right Hip abductor (sideways)
     * R_HFE - 11 - Right Hip (bends)
     * R_KFE - 13 - Right Knee
     * R_AFE - 15 - Right Ankle (bends)
     * R_AR  - 17 - Right Foot (sideways rotate)
     * L_SAA -  2 - Left Shoulder (rotates)
     * L_SFE -  4 - Left Biceps (bends)
     * L_EB  -  6 - Left Elbow
     * L_HAA -  8 - Left Hip (rotates)
     * L_HR  - 10 - Left Hip abductor (sideways)
     * L_HFE - 12 - Left Hip (bends)
     * L_KFE - 14 - Left Knee
     * L_AFE - 16 - Left Ankle (bends)
     * L_AR  - 18 - Left Foot (sideways rotate)
     *
     */

    const std::map<std::string, double> init_values {
        {"R_SAA", 0.0},
        {"R_SFE", 0.0},
        {"R_EB",  0.0},
        {"R_HAA", 0.0},
        {"R_HR",  0.0},
        {"R_HFE", 0.0},
        {"R_KFE", 0.0},
        {"R_AFE", 0.0},
        {"R_AR",  0.0},
        {"L_SAA", 0.0},
        {"L_SFE", 0.0},
        {"L_EB",  0.0},
        {"L_HAA", 0.0},
        {"L_HR",  0.0},
        {"L_HFE", 0.0},
        {"L_KFE", 0.0},
        {"L_AFE", 0.0},
        {"L_AR",  0.0},
    };

    // assign values to group
    bool success = group->setJointValueTarget(init_values);
    if(!success) std::cout << "setJointValueTarget() failed" << std::endl;

    // this will plan and execute in one step
    success = group->move();
    if(!success) std::cout << "moveto_default_state() failed" << std::endl;

    return *this;
}

void Core::updateTF()
{
    listener->lookupTransform("/base_link", "/R_foot_pad", ros::Time(0), *transforms[Limb::RIGHT_FOOT]);
    listener->lookupTransform("/base_link", "/L_foot_pad", ros::Time(0), *transforms[Limb::LEFT_FOOT]);
    listener->lookupTransform("/base_link", "/R_forearm", ros::Time(0), *transforms[Limb::RIGHT_HAND]);
    listener->lookupTransform("/base_link", "/L_forearm", ros::Time(0), *transforms[Limb::LEFT_HAND]);
}

bool Core::get_isstanding() {
    return isstanding;
}

void Core::set_isstanding(bool yes_no) {
    isstanding=yes_no;
}

