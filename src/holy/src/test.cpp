#include "ros/ros.h"
#include <math.h>
#include <geometry_msgs/Twist.h>

const double PI = 3.14159265359;

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
	// Fährt Nikolaus Haus ab
	// Dummy Befehl, tut nichts bei der turtle

	cmd.linear.x = velocity;
    chatter_pub.publish(cmd);
    ros::Duration(1).sleep();
    reset_cmd(cmd);
    // 90 Grad drehen
	cmd.angular.z = PI/2;
    chatter_pub.publish(cmd);
    ros::Duration(1).sleep();
    reset_cmd(cmd);
    // nach oben
    cmd.linear.x = +velocity;
    chatter_pub.publish(cmd);
    ros::Duration(1).sleep();
    reset_cmd(cmd);
    // 90 Grad drehen nach rechts
	cmd.angular.z = -PI/2;
    chatter_pub.publish(cmd);
    ros::Duration(1).sleep();
    reset_cmd(cmd);
    // nach rechts
    cmd.linear.x = +velocity;
    chatter_pub.publish(cmd);
    ros::Duration(1).sleep();
    reset_cmd(cmd);
    // 135 Grad drehen nach rechts
	cmd.angular.z = -PI/2 - PI/4;
    chatter_pub.publish(cmd);
    ros::Duration(1).sleep();
    reset_cmd(cmd);
    // nach links unten
    cmd.linear.x = +velocity*sqrt(2);
    chatter_pub.publish(cmd);
    ros::Duration(1).sleep();
    reset_cmd(cmd);
    // 135 Grad drehen nach links
	cmd.angular.z = PI/4+PI/2;
    chatter_pub.publish(cmd);
    ros::Duration(1).sleep();
    reset_cmd(cmd);
    // nach rechts
    cmd.linear.x = +velocity;
    chatter_pub.publish(cmd);
    ros::Duration(1).sleep();
    reset_cmd(cmd);
    // 135 Grad drehen nach links
	cmd.angular.z = PI/4+PI/2;
    chatter_pub.publish(cmd);
    ros::Duration(1).sleep();
    reset_cmd(cmd);
    // nach links oben
    cmd.linear.x = +velocity*sqrt(2);
    chatter_pub.publish(cmd);
    ros::Duration(1).sleep();
    reset_cmd(cmd);
    // 90 grad nach rechts
	cmd.angular.z = -PI/2;
    chatter_pub.publish(cmd);
    ros::Duration(1).sleep();
    reset_cmd(cmd);
    // nach rechts oben
    cmd.linear.x = +velocity*sqrt(2)/2;
    chatter_pub.publish(cmd);
    ros::Duration(1).sleep();
    reset_cmd(cmd);
    // 90 grad nach rechts
	cmd.angular.z = -PI/2;
    chatter_pub.publish(cmd);
    ros::Duration(1).sleep();
    reset_cmd(cmd);
    // nach rechts oben
    cmd.linear.x = +velocity*sqrt(2)/2;
    chatter_pub.publish(cmd);
    ros::Duration(1).sleep();
    reset_cmd(cmd);
    // 45 grad nach rechts
	cmd.angular.z = -PI/4;
    chatter_pub.publish(cmd);
    ros::Duration(1).sleep();
    reset_cmd(cmd);
    // nach unten
    cmd.linear.x = +velocity;
    chatter_pub.publish(cmd);
    ros::Duration(1).sleep();
    reset_cmd(cmd);
    // 90 grad nach links
	cmd.angular.z = +PI/2;
    chatter_pub.publish(cmd);
    ros::Duration(1).sleep();
    reset_cmd(cmd);
    ros::spinOnce();
	//loop_rate.sleep();
  }

  return 0;
}
