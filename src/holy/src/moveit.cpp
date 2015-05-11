#include <cmath>

#include <vector>
#include <unordered_map>

#include <ros/ros.h>
#include <bioloid_interface/bioloid_msg.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>

#include <moveit_msgs/MoveGroupActionResult.h>


/* L_EB  - Left Elbow
 * L_SFE - Left Biceps (bends)
 * L_SAA - Left Shoulder (rotates)
 * L_HAA - Left Hip (rotates)
 * L_HR  - Left Hip (bends)
 * L_HFE - Left Hip abductor (sideways)
 * L_KFE - Left Knee
 * L_AFE - Left Ankle (bends)
 * L_AR  - Left Foot (sideways rotate)
 * R_EB  - Right Elbow
 * R_SFE - Right Biceps (bends)
 * R_SAA - Right Shoulder (rotates)
 * R_HAA - Right Hip (rotates)
 * R_HR  - Right Hip (bends)
 * R_HFE - Right Hip abductor (sideways)
 * R_KFE - Right Knee
 * R_AFE - Right Ankle (bends)
 * R_AR  - Right Foot (sideways rotate)
 */

ros::Publisher pub;
ros::Subscriber sub;
sensor_msgs::JointState js;

void controllerResultCallback(const moveit_msgs::MoveGroupActionResultConstPtr& msg)
{
    auto jn = msg->result.planned_trajectory.joint_trajectory.joint_names;
    auto jp = msg->result.planned_trajectory.joint_trajectory.points;

    ROS_INFO("%s", msg->status.text.c_str());
    ROS_INFO("Got %d joints. Joint 0 has %d points", static_cast<int>(jn.size()), static_cast<int>(jp[0].positions.size()));

    sensor_msgs::JointState js;
    js.header.stamp = ros::Time::now();
    js.header.frame_id = "/world";
    for (int i=0; i<jn.size(); ++i) {
        js.name.push_back(jn.at(i));
        js.position.push_back(jp.at(i).positions.at(0));
        js.effort.push_back(-1);
    }
    pub.publish(js);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "moveit_controller");
    ros::NodeHandle n;

    pub = n.advertise<sensor_msgs::JointState>("bioloid_interface/command", 1000);
    sub = n.subscribe("move_group/result", 1000, controllerResultCallback);

    ros::Rate rate(20);

    while (ros::ok()) {
        rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
