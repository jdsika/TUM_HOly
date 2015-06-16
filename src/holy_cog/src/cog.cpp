
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <string>

int main(int argc, char **argv)
{

    ros::init(argc, argv, "holy_cog");
    ros::NodeHandle node_handle();

    std::map<int, tf::StampedTransform> transforms
    {
        {14, tf::StampedTransform()}, // L_calf
        {16, tf::StampedTransform()}, // L_foot
        {18, tf::StampedTransform()}, // L_foot_pad
        {6, tf::StampedTransform()},  // L_forearm
        {8, tf::StampedTransform()},  // L_hip
        {10, tf::StampedTransform()}, // L_hip_split
        {2, tf::StampedTransform()},  // L_shoulder
        {4, tf::StampedTransform()},  // L_upper_arm
        {12, tf::StampedTransform()}, // L_upper_leg
        {13, tf::StampedTransform()}, // R_...
        {15, tf::StampedTransform()},
        {17, tf::StampedTransform()},
        {5, tf::StampedTransform()},
        {7, tf::StampedTransform()},
        {9, tf::StampedTransform()},
        {1, tf::StampedTransform()},
        {3, tf::StampedTransform()},
        {11, tf::StampedTransform()},
    };

    tf::TransformListener listener;

    ROS_INFO("Waiting for TF to appear");
    listener.waitForTransform("/base_link", "/R_foot_pad",ros::Time(0), ros::Duration(5));

    tf::TransformBroadcaster broadcaster;

    ros::Rate rate(100);

    while(ros::ok()) {

        // Update TFs
        listener.lookupTransform("/base_link", "/L_calf", ros::Time(0), transforms[14]);
        listener.lookupTransform("/base_link", "/L_foot", ros::Time(0), transforms[16]);
        listener.lookupTransform("/base_link", "/L_foot_pad", ros::Time(0), transforms[18]);
        listener.lookupTransform("/base_link", "/L_forearm", ros::Time(0), transforms[6]);
        listener.lookupTransform("/base_link", "/L_hip", ros::Time(0), transforms[8]);
        listener.lookupTransform("/base_link", "/L_hip_split", ros::Time(0), transforms[10]);
        listener.lookupTransform("/base_link", "/L_shoulder", ros::Time(0), transforms[2]);
        listener.lookupTransform("/base_link", "/L_upper_arm", ros::Time(0), transforms[4]);
        listener.lookupTransform("/base_link", "/L_upper_leg", ros::Time(0), transforms[12]);
        listener.lookupTransform("/base_link", "/R_calf", ros::Time(0), transforms[13]);
        listener.lookupTransform("/base_link", "/R_foot", ros::Time(0), transforms[15]);
        listener.lookupTransform("/base_link", "/R_foot_pad", ros::Time(0), transforms[17]);
        listener.lookupTransform("/base_link", "/R_forearm", ros::Time(0), transforms[5]);
        listener.lookupTransform("/base_link", "/R_hip", ros::Time(0), transforms[7]);
        listener.lookupTransform("/base_link", "/R_hip_split", ros::Time(0), transforms[9]);
        listener.lookupTransform("/base_link", "/R_shoulder", ros::Time(0), transforms[1]);
        listener.lookupTransform("/base_link", "/R_upper_arm", ros::Time(0), transforms[3]);
        listener.lookupTransform("/base_link", "/R_upper_leg", ros::Time(0), transforms[11]);

        // calculate middle between two joints, which is the center of a link
        tf::Vector3 link_center_1_3 = (transforms[1].getOrigin() + transforms[3].getOrigin() ) / 2;
        tf::Vector3 link_center_3_5 = (transforms[3].getOrigin() + transforms[5].getOrigin() ) / 2;
        tf::Vector3 link_center_2_4 = (transforms[2].getOrigin() + transforms[4].getOrigin() ) / 2;
        tf::Vector3 link_center_4_6 = (transforms[4].getOrigin() + transforms[6].getOrigin() ) / 2;
        tf::Vector3 link_center_7_9 = (transforms[7].getOrigin() + transforms[9].getOrigin() ) / 2;
        tf::Vector3 link_center_9_11 = (transforms[9].getOrigin() + transforms[11].getOrigin() ) / 2;
        tf::Vector3 link_center_11_13 = (transforms[11].getOrigin() + transforms[13].getOrigin() ) / 2;
        tf::Vector3 link_center_13_15 = (transforms[13].getOrigin() + transforms[15].getOrigin() ) / 2;
        tf::Vector3 link_center_15_17 = (transforms[15].getOrigin() + transforms[17].getOrigin() ) / 2;
        tf::Vector3 link_center_8_10 = (transforms[8].getOrigin() + transforms[10].getOrigin() ) / 2;
        tf::Vector3 link_center_10_12 = (transforms[10].getOrigin() + transforms[12].getOrigin() ) / 2;
        tf::Vector3 link_center_12_14 = (transforms[12].getOrigin() + transforms[14].getOrigin() ) / 2;
        tf::Vector3 link_center_14_16 = (transforms[14].getOrigin() + transforms[16].getOrigin() ) / 2;
        tf::Vector3 link_center_16_18 = (transforms[16].getOrigin() + transforms[18].getOrigin() ) / 2;
        tf::Vector3 link_center_0_x (0,0,0);
        tf::Vector3 link_center_5_x = transforms[5].getOrigin() + transforms[5].getBasis() * tf::Vector3(0, 0,-0.039);
        tf::Vector3 link_center_6_x = transforms[6].getOrigin() + transforms[6].getBasis() * tf::Vector3(0, 0,-0.039);
        tf::Vector3 link_center_17_x = transforms[17].getOrigin() + transforms[17].getBasis() * tf::Vector3(0, 0, -0.02);
        tf::Vector3 link_center_18_x = transforms[18].getOrigin() + transforms[18].getBasis() * tf::Vector3(0, 0, -0.02);

        // weight with link weights (gram)
        link_center_1_3   *= 10;
        link_center_3_5   *= 70;
        link_center_2_4   *= 10;
        link_center_4_6   *= 70;
        link_center_7_9   *= 14;
        link_center_9_11  *= 122;
        link_center_11_13 *= 28;
        link_center_13_15 *= 74;
        link_center_15_17 *= 124;
        link_center_8_10  *= 14;
        link_center_10_12 *= 122;
        link_center_12_14 *= 28;
        link_center_14_16 *= 74;
        link_center_16_18 *= 124;
        link_center_0_x   *= 516;
        link_center_5_x   *= 76;
        link_center_6_x   *= 76;
        link_center_17_x  *= 34;
        link_center_18_x  *= 34;
        double total_weight = 1620;

        // calculate average
        tf::Vector3 avg = ( link_center_1_3 + link_center_3_5 + link_center_2_4 + link_center_4_6 + link_center_7_9 + link_center_9_11 + link_center_11_13 + link_center_13_15 + link_center_15_17 + link_center_8_10 + link_center_10_12 + link_center_12_14 + link_center_14_16 + link_center_16_18 + link_center_0_x + link_center_5_x + link_center_6_x + link_center_17_x + link_center_18_x ) / total_weight;

        // broadcast resulting TF
        tf::Transform normalTransform;
        normalTransform.setOrigin(avg);
        tf::Quaternion q;
        q.setRPY(0,0,0);
        normalTransform.setRotation(q);
        tf::StampedTransform stampedTransform (normalTransform, ros::Time::now(), "/base_link", "COG");
        broadcaster.sendTransform(stampedTransform);

        ros::spinOnce();
        rate.sleep();

    }
    return 0;

}
