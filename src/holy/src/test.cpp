#include "ros/ros.h"

#include <geometry_msgs/Twist.h>

void reset_cmd(geometry_msgs::Twist cmd) {
	cmd.linear.x=0;
	cmd.linear.y=0;
	cmd.angular.x=0;
	cmd.angular.y=0;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "turtle_house");
  ros::NodeHandle n;
  // create Twist message
  geometry_msgs::Twist cmd;
  //ros::Rate loop_rate(5);

  ros::Publisher chatter_pub = n.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1000);

  while (ros::ok()) {
	cmd.linear.y = 0.5;
    chatter_pub.publish(cmd);
    ros::Duration(1).sleep();
    reset_cmd(cmd);

    ros::spinOnce();
	//loop_rate.sleep();
  }

  return 0;
}
