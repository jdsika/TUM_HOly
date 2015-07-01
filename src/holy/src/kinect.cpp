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
    kin_listener->lookupTransform("tracker/user_1/torso", "tracker/user_1/left_foot", ros::Time(0), kin_transforms[Core::Limb::LEFT_FOOT]);
    kin_listener->lookupTransform("tracker/user_1/torso", "tracker/user_1/right_foot", ros::Time(0), kin_transforms[Core::Limb::RIGHT_FOOT]);
    kin_listener->lookupTransform("tracker/user_1/left_shoulder", "tracker/user_1/left_hand", ros::Time(0), kin_transforms[Core::Limb::LEFT_HAND]);
    kin_listener->lookupTransform("tracker/user_1/right_shoulder", "tracker/user_1/right_hand", ros::Time(0), kin_transforms[Core::Limb::RIGHT_HAND]);
}

double Kinect::compute_norm(tf::StampedTransform &frame1) {
    return sqrt(pow(frame1.getOrigin().x(),2)+pow(frame1.getOrigin().y(),2)+pow(frame1.getOrigin().z(),2));
}

void Kinect::human_to_robopose()
{
    // Arms
    // Get orientation from Kinect
    double alroll,alpitch=0,alyaw=0,lnorm;
    double arroll,arpitch=0,aryaw=0,rnorm;
    lnorm=compute_norm(kin_transforms[Core::Limb::LEFT_HAND]);
    rnorm=compute_norm(kin_transforms[Core::Limb::RIGHT_HAND]);
    alroll= acos(kin_transforms[Core::Limb::LEFT_HAND].getOrigin().z()/lnorm) -M_PI/2;
    arroll= acos(kin_transforms[Core::Limb::RIGHT_HAND].getOrigin().z()/rnorm) -M_PI/2;
    //alroll=Poses::d2r(30)-1*alroll;
    alyaw= 0;//acos(kin_transforms[Core::Limb::LEFT_HAND].getOrigin().z());
    alpitch= acos(kin_transforms[Core::Limb::LEFT_HAND].getOrigin().x()/lnorm) -M_PI/2;
    arpitch= -(acos(-kin_transforms[Core::Limb::RIGHT_HAND].getOrigin().x()/rnorm) -M_PI/2);
    if (kin_transforms[Core::Limb::LEFT_HAND].getOrigin().y()>0) {
        alpitch= M_PI/2 + (M_PI/2 -alpitch);
    }
    if (kin_transforms[Core::Limb::RIGHT_HAND].getOrigin().y()>0) {
        arpitch= -M_PI/2 + (-M_PI/2 -arpitch);
    }
    //ROS_INFO("%lf,%lf,%lf",kin_transforms[Core::Limb::LEFT_HAND].getOrigin().x(),kin_transforms[Core::Limb::LEFT_HAND].getOrigin().y(),kin_transforms[Core::Limb::LEFT_HAND].getOrigin().z());
    ROS_INFO("%lf,%lf,%lf",Poses::r2d(arroll),Poses::r2d(arpitch),Poses::r2d(aryaw));
    //ROS_INFO("%lf,%lf,%lf",alroll,alpitch,alyaw);
    pose_from_kinect = RoboPose (std::vector<LimbPose> {
                           //changed to have lower arms
                                           LimbPose(Core::Limb::LEFT_HAND,  alroll , alpitch, alyaw, 0, 0, 0),
                                           LimbPose(Core::Limb::RIGHT_HAND, arroll,  arpitch, 0,  0, 0, 0),
                                       } , "pose_kinect");
}

void Kinect::StateMachine() {
    // Wait until new Kinect TF data is available
    bool success= kin_listener->waitForTransform("tracker/user_1/torso", "tracker/user_1/left_foot",ros::Time(0), ros::Duration(0.5));
    success= success*kin_listener->waitForTransform("tracker/user_1/torso", "tracker/user_1/right_foot",ros::Time(0), ros::Duration(0.5));
    success= success*kin_listener->waitForTransform("tracker/user_1/torso", "tracker/user_1/left_hand",ros::Time(0), ros::Duration(0.5));
    success= success*kin_listener->waitForTransform("tracker/user_1/torso", "tracker/user_1/right_hand",ros::Time(0), ros::Duration(0.5));
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

        if (kinect_fsm == Kinect_FSM::STAND) {
            pose_dual_stand=Poses::pose_default+pose_from_kinect;
            core->setPoseTarget(pose_dual_stand).move(5.0);
        }
        else if (kinect_fsm == Kinect_FSM::RBALANCE) {


        }
        else if (kinect_fsm == Kinect_FSM::LBALANCE) {
        }
    }


}

void Kinect::init_StateMachine() {
    // Init to first element in FSM
    kinect_fsm = Kinect_FSM::STAND;
}
