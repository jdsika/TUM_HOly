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

    sensor_msgs::JointState js;

    js.header.frame_id = "/world";

    js.name.push_back("L_SAA");
    js.name.push_back("R_SAA");
    js.name.push_back("L_SFE");
    js.name.push_back("R_SFE");
    js.position.push_back(1);
    js.position.push_back(1);
    js.position.push_back(0);
    js.position.push_back(0);

    js.effort.resize(4);
    std::fill(js.effort.begin(), js.effort.end(), 0);


    while (ros::ok()) {

        js.header.stamp = ros::Time::now();
        js.position[2] = 1;
        js.position[3] = 1;
        pub.publish(js);
        ros::Duration(1.0).sleep();

        js.header.stamp = ros::Time::now();
        js.position[2] = -1;
        js.position[3] = -1;
        pub.publish(js);
        ros::Duration(1.0).sleep();

        ros::spinOnce();
    }

    return 0;
}
