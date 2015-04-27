#include "ros/ros.h"

#include <cmath>

#include <vector>

#include <bioloid_interface/bioloid_msg.h>
#include <sensor_msgs/JointState.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "wave");
    ros::NodeHandle n;

    ros::Publisher pub = n.advertise<sensor_msgs::JointState>("bioloid_interface/command", 1000);

    ros::Rate rate(20);

    sensor_msgs::JointState js;

    js.header.frame_id = "/world";

    // Add joints to vector
    js.name.push_back("L_SAA");
    js.name.push_back("R_SAA");
    js.name.push_back("L_SFE");
    js.name.push_back("R_SFE");
    js.name.push_back("L_EB");
    js.name.push_back("R_EB");
    js.position.push_back(1);
    js.position.push_back(1);
    js.position.push_back(0);
    js.position.push_back(0);
    js.position.push_back(0);
    js.position.push_back(0);

    // Set Bioloid effort
    js.effort.resize(6);
    std::fill(js.effort.begin(), js.effort.end(), 200);

    while (ros::ok()) {

        js.header.stamp = ros::Time::now();
        double t = ros::Time::now().toSec();
        js.position[0] = sin(t);
        js.position[1] = sin(t+M_PI);
        js.position[2] = sin(t);
        js.position[3] = sin(t+M_PI);
        js.position[4] = sin(t+M_PI_2);
        js.position[5] = -sin(t+M_PI+M_PI_2);
        pub.publish(js);

        rate.sleep();

        ros::spinOnce();
    }

    return 0;
}
