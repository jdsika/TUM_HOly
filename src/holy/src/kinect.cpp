#include "kinect.h"

#include "core/core.h"
#include "poses/poses.h"
#include "poses/robopose.h"
#include <tf/transform_listener.h>

#define DEBUG 1

Kinect::Kinect(Core *core) : core{core}
{

    init_StateMachine();
    // Always possible to switch mode
    core->set_isstanding(true);
    // Wait for TF to appear
    kin_listener= new tf::TransformListener;
    kin_transforms[Core::Limb::RIGHT_FOOT] = tf::StampedTransform(); // Right Foot
    kin_transforms[Core::Limb::LEFT_FOOT] = tf::StampedTransform(); // etc
    kin_transforms[Core::Limb::RIGHT_HAND] = tf::StampedTransform();
    kin_transforms[Core::Limb::LEFT_HAND] = tf::StampedTransform();

}

Kinect::~Kinect()
{
    delete kin_listener;
}


void Kinect::updateTF()
{

    // Left right is swapped in Kinect frame!
    kin_listener->lookupTransform("tracker/user_1/left_hip", "tracker/user_1/left_foot", ros::Time(0), kin_transforms[Core::Limb::LEFT_FOOT]);
    kin_listener->lookupTransform("tracker/user_1/right_hip", "tracker/user_1/right_foot", ros::Time(0), kin_transforms[Core::Limb::RIGHT_FOOT]);
    kin_listener->lookupTransform("tracker/user_1/left_elbow", "tracker/user_1/left_hand", ros::Time(0), kin_transforms[Core::Limb::LEFT_HAND]);
    kin_listener->lookupTransform("tracker/user_1/right_elbow", "tracker/user_1/right_hand", ros::Time(0), kin_transforms[Core::Limb::RIGHT_HAND]);
    kin_listener->lookupTransform("tracker/user_1/left_shoulder", "tracker/user_1/left_elbow", ros::Time(0), left_shoulder_hand);
    kin_listener->lookupTransform("tracker/user_1/right_shoulder", "tracker/user_1/right_elbow", ros::Time(0), right_shoulder_hand);
}

double Kinect::compute_norm(tf::StampedTransform &frame1) {
    return sqrt(pow(frame1.getOrigin().x(),2)+pow(frame1.getOrigin().y(),2)+pow(frame1.getOrigin().z(),2));
}

void Kinect::human_to_robopose()
{
    // ARMS

    // Get orientation from Kinect
    // left arm
    double alroll,alpitch=0,lnorm;
    lnorm=compute_norm(kin_transforms[Core::Limb::LEFT_HAND]);
    alroll= acos(kin_transforms[Core::Limb::LEFT_HAND].getOrigin().z()/lnorm) -M_PI/2;
    alpitch= acos(kin_transforms[Core::Limb::LEFT_HAND].getOrigin().x()/lnorm) -M_PI/2;
    if (kin_transforms[Core::Limb::LEFT_HAND].getOrigin().y()>0) {
        alpitch= M_PI/2 + (M_PI/2 -alpitch);
    }
    // Compute elbow angle:
    double scalarproduct, angle;
    scalarproduct=left_shoulder_hand.getOrigin().x()*kin_transforms[Core::Limb::LEFT_HAND].getOrigin().x()+
                    left_shoulder_hand.getOrigin().y()*kin_transforms[Core::Limb::LEFT_HAND].getOrigin().y()+
                    left_shoulder_hand.getOrigin().z()*kin_transforms[Core::Limb::LEFT_HAND].getOrigin().z();
    angle=acos(scalarproduct/(lnorm*compute_norm(left_shoulder_hand)));
    core->set_left_elbow_min_max(angle);
    // right arm
    double arroll,arpitch=0,rnorm;
    rnorm=compute_norm(kin_transforms[Core::Limb::RIGHT_HAND]);
    arroll= acos(kin_transforms[Core::Limb::RIGHT_HAND].getOrigin().z()/rnorm) -M_PI/2;
    arpitch= -(acos(-kin_transforms[Core::Limb::RIGHT_HAND].getOrigin().x()/rnorm) -M_PI/2);
    if (kin_transforms[Core::Limb::RIGHT_HAND].getOrigin().y()>0) {
        arpitch= -M_PI/2 + (-M_PI/2 -arpitch);
    }
    // Compute elbow angle:
    scalarproduct=right_shoulder_hand.getOrigin().x()*kin_transforms[Core::Limb::RIGHT_HAND].getOrigin().x()+
                    right_shoulder_hand.getOrigin().y()*kin_transforms[Core::Limb::RIGHT_HAND].getOrigin().y()+
                    right_shoulder_hand.getOrigin().z()*kin_transforms[Core::Limb::RIGHT_HAND].getOrigin().z();
    angle=acos(scalarproduct/(rnorm*compute_norm(right_shoulder_hand)));
    core->set_right_elbow_min_max(angle);
    //ROS_INFO("%lf,%lf,%lf",kin_transforms[Core::Limb::LEFT_HAND].getOrigin().x(),kin_transforms[Core::Limb::LEFT_HAND].getOrigin().y(),kin_transforms[Core::Limb::LEFT_HAND].getOrigin().z());
    //ROS_INFO("%lf,%lf,%lf",Poses::r2d(arroll),Poses::r2d(arpitch),Poses::r2d(aryaw));
    //ROS_INFO("%lf,%lf,%lf",alroll,alpitch,alyaw);




    // LEGS
    // get Z of both legs and compute mean and map to robot values --> possible to crouch
    double max_z=0.05;
    double z_l,z_r,z_kin, z_robo, z_kin_max=-0.20, z_kin_min=-0.70;
    z_l=kin_transforms[Core::Limb::LEFT_FOOT].getOrigin().y();
    z_r=kin_transforms[Core::Limb::RIGHT_FOOT].getOrigin().y();
    z_kin=(z_l+z_r)/2;
    if (z_kin < z_kin_min) z_kin=z_kin_min;
    if (z_kin > z_kin_max) z_kin=z_kin_max;
    z_robo=max_z*(1-(z_kin_max-z_kin)/(z_kin_max-z_kin_min));
    ROS_INFO("%lf,%lf",z_kin,z_robo);
    pose_from_kinect = RoboPose (std::vector<LimbPose> {
                           //changed to have lower arms
                                           LimbPose(Core::Limb::LEFT_HAND,  alroll , alpitch-Poses::d2r(10), 0, 0, 0, 0),
                                           LimbPose(Core::Limb::RIGHT_HAND, arroll,  arpitch+Poses::d2r(10), 0, 0, 0, 0),
                                           LimbPose(Core::Limb::LEFT_FOOT,  0, 0, 0, -0.01, 0, z_robo),
                                           LimbPose(Core::Limb::RIGHT_FOOT, 0, 0, 0, 0.01, 0, z_robo),
                                       } , "pose_kinect");
}

void Kinect::StateMachine() {
    // Wait until new Kinect TF data is available
    bool success= kin_listener->waitForTransform("tracker/user_1/torso", "tracker/user_1/left_foot",ros::Time(0), ros::Duration(0.5));
    success= success*kin_listener->waitForTransform("tracker/user_1/torso", "tracker/user_1/right_foot",ros::Time(0), ros::Duration(0.5));
    success= success*kin_listener->waitForTransform("tracker/user_1/left_elbow", "tracker/user_1/left_hand",ros::Time(0), ros::Duration(0.5));
    success= success*kin_listener->waitForTransform("tracker/user_1/right_elbow", "tracker/user_1/right_hand",ros::Time(0), ros::Duration(0.5));
    if (!success) {
        ROS_INFO("no Kinect data found");
    }
    else {
        updateTF();


        human_to_robopose();
        // update parameters
        //kinect_poses.update();

        // Construct a Robopose based on Kinect input and send it to robot
        // 3 Modes: Dual support, left and right support
        // Automatically changed due to height of human foot ( left foot raised --> lean right, left foot is movable)
        // In Stand mode only hands can be moved

        pose_dual_stand=Poses::pose_default+pose_from_kinect;
        core->setPoseTarget(pose_dual_stand).move(5.0,false);

    }


}

void Kinect::init_StateMachine() {
    // Init to first element in FSM
    kinect_fsm = Kinect_FSM::STAND;
}
