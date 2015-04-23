#include "ros/ros.h"

#include <geometry_msgs/Twist.h>

void reset_cmd(geometry_msgs::Twist& cmd) {
	cmd.linear.x=0;
	cmd.angular.z=0;

}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "turtle_house");
  ros::NodeHandle n;
  // create Twist message
  geometry_msgs::Twist cmd;
  //ros::Rate loop_rate(5);
  ros::Publisher chatter_pub = n.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1000);
  double velocity=1;
  while (ros::ok()) {
	// 90 grad drehen und nach oben nach oben
	/*ros::Duration(1).sleep();
	ROS_INFO("90 Grad drehen");
	reset_cmd(cmd);
	ROS_INFO("%f", cmd.linear.x);
	cmd.angular.z=velocity;
    chatter_pub.publish(cmd);
    ros::Duration(6).sleep();
    reset_cmd(cmd);*/
	cmd.linear.x = velocity;
    chatter_pub.publish(cmd);
    ros::Duration(1).sleep();
    reset_cmd(cmd);
    // nach rechts
    cmd.linear.x = -velocity;
    chatter_pub.publish(cmd);
    ros::Duration(1).sleep();
    reset_cmd(cmd);
    chatter_pub.publish(cmd);
    /*// nach unten
    cmd.linear.y = -velocity;
    chatter_pub.publish(cmd);
    ros::Duration(1).sleep();
    reset_cmd(cmd);
    // schräg links unten
    cmd.linear.y = -velocity;
    cmd.linear.x=velocity;
    chatter_pub.publish(cmd);
    ros::Duration(1).sleep();
    reset_cmd(cmd);
    // nach rechts
    cmd.linear.x=velocity;
    chatter_pub.publish(cmd);
    ros::Duration(1).sleep();
    reset_cmd(cmd);
    // schräg links oben
    cmd.linear.y = velocity;
    cmd.linear.x = -velocity;
    chatter_pub.publish(cmd);
    ros::Duration(1).sleep();
    reset_cmd(cmd);
    // schräg rechts oben
    cmd.linear.y = velocity;
    cmd.linear.x = velocity;
    chatter_pub.publish(cmd);
    ros::Duration(1).sleep();
    reset_cmd(cmd);
    // schräg rechts unten
    cmd.linear.y = -velocity;
    cmd.linear.x = velocity;
    chatter_pub.publish(cmd);
    ros::Duration(1).sleep();
    reset_cmd(cmd);
    // unten
    cmd.linear.y = -velocity;
    chatter_pub.publish(cmd);
    ros::Duration(1).sleep();
    reset_cmd(cmd);
    ros::spinOnce();
	//loop_rate.sleep();*/
  }

  return 0;
}
