#include "core.h"
#include "walk.h"

#include <geometry_msgs/Pose.h>

Walk::Walk(Core *core) : core{core}
{

}

Walk::~Walk()
{

}

Walk::pose Walk::getCurrentPose(Core::Limb limb)
{

    geometry_msgs::Pose currPos;
    tf::StampedTransform* transf = core->getTF(limb);
    tf::Matrix3x3 matOrient;
    matOrient.setRotation(transf->getRotation());
    tf::Quaternion quat;
    matOrient.getRotation(quat);
    currPos.orientation.x = quat.getX();
    currPos.orientation.y = quat.getY();
    currPos.orientation.z = quat.getZ();
    currPos.orientation.w = quat.getW();

    // translation addieren
    currPos.position.x = static_cast<double>(transf->getOrigin().x());
    currPos.position.y = static_cast<double>(transf->getOrigin().y());
    currPos.position.z = static_cast<double>(transf->getOrigin().z());




//    geometry_msgs::PoseStamped poseStamped = core->getMoveGroup().getCurrentPose(Core::getLimbString(limb));

    double r,p,y;
    tf::Matrix3x3(tf::Quaternion(currPos.orientation.x,
                                 currPos.orientation.y,
                                 currPos.orientation.z,
                                 currPos.orientation.w)
                  ).getEulerYPR(y, p, r);
//    tf::Matrix3x3(tf::Quaternion(currPos.pose.orientation.x,
//                                 currPos.pose.orientation.y,
//                                 currPos.pose.orientation.z,
//                                 currPos.pose.orientation.w)
//                  ).getEulerYPR(y, p, r);

    struct pose currentPose = {
        r, p, y,
//        currPos.pose.position.x,
//        currPos.pose.position.y,
//        currPos.pose.position.z
        currPos.position.x,
        currPos.position.y,
        currPos.position.z
    };
    return currentPose;
}

const geometry_msgs::Pose Walk::pose::toGeoPose() const
{
    tf::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    geometry_msgs::Pose gpose;
    gpose.orientation.x = q.getX();
    gpose.orientation.y = q.getY();
    gpose.orientation.z = q.getZ();
    gpose.orientation.w = q.getW();
    gpose.position.x = x;
    gpose.position.y = y;
    gpose.position.z = z;
    return gpose;
}
