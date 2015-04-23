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
    //    std::fill(js.position.begin(), js.position.end(), 0);

    js.header.frame_id = "/world";

    js.name.resize(1);
    js.name[0] = "L_SFE";

    js.position.resize(1);
    js.position[0] = 0;

    js.effort.resize(1);
    js.effort[0] = 0;


    while (ros::ok()) {

        js.header.stamp = ros::Time::now();
        js.position[0] = 1;
        pub.publish(js);
        ros::Duration(1.0).sleep();

        js.header.stamp = ros::Time::now();
        js.position[0] = 0.5;
        pub.publish(js);
        ros::Duration(1.0).sleep();

        ros::spinOnce();
    }

    return 0;
}
