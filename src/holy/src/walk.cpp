#include <cmath>

#include <vector>
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <moveit/move_group_interface/move_group.h>
#include <geometry_msgs/Pose.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <tf/transform_listener.h>

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



int main(int argc, char **argv)
{
    ros::init(argc, argv, "holy_walk");
    ros::NodeHandle n;
    ros::Publisher display_publisher = n.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    moveit::planning_interface::MoveGroup group("All");

    //moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit_msgs::DisplayTrajectory display_trajectory;
    geometry_msgs::Pose target_pose1;
    moveit::planning_interface::MoveGroup::Plan my_plan;

    tf::TransformListener listener;
    tf::StampedTransform transform;
    sleep(1.0);
    try{
     listener.lookupTransform("/base_link", "/R_foot_pad",
                              ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
     ROS_ERROR("%s",ex.what());
     ros::Duration(1.0).sleep();
    }
    ROS_INFO("%f",(double)transform.getOrigin().z());
    tf::Matrix3x3 m,m2;
    m.setRPY(0,-25*M_PI/180.0,0*M_PI/180.0);
    m2.setRotation(transform.getRotation());
    m=m*m2;
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    std::cout << roll<< std::endl << pitch<< std::endl << yaw << std::endl;
    tf::Quaternion quat;
    m.getRotation(quat);
    target_pose1.orientation.x = quat.getX();//(double)transform.getRotation().getX();
    target_pose1.orientation.y = quat.getY();//(double)transform.getRotation().getY();
    target_pose1.orientation.z = quat.getZ();//(double)transform.getRotation().getZ();
    target_pose1.orientation.w = quat.getW();//(double)transform.getRotation().getW();

    target_pose1.position.x = (double)transform.getOrigin().x();
    target_pose1.position.y = (double)transform.getOrigin().y();
    target_pose1.position.z = (double)transform.getOrigin().z() +0.00;
    group.setGoalTolerance(0.01);
    group.setPoseTarget(target_pose1, "R_foot_pad");
    ROS_INFO("Eumel");
    //bool success = group.plan(my_plan);
    //ROS_INFO("%i",success);
    group.move();
   // ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");
    /* Sleep to give Rviz time to visualize the plan. */
    //sleep(5.0);

    if (1)
    {
      ROS_INFO("Visualizing plan 1 (again)");
      display_trajectory.trajectory_start = my_plan.start_state_;
      display_trajectory.trajectory.push_back(my_plan.trajectory_);
      display_publisher.publish(display_trajectory);
      /* Sleep to give Rviz time to visualize the plan. */
      //sleep(5.0);
    }
    ros::Rate rate(20);

    while (ros::ok()) {

        rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
