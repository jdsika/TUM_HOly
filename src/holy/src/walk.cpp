#include "core.h"
#include "walk.h"

#include <geometry_msgs/Pose.h>

Walk::Walk(Core *core) : core{core}
{
    core->moveto_default_state();
    // wenn startpos erreicht, offset-TF speichern
}

Walk::~Walk()
{

}

Walk::pose Walk::getCurrentPose(Core::Limb limb, bool debugOut)
{

    geometry_msgs::Pose currPos;
    tf::StampedTransform* transf = core->getTF(limb);
    tf::Matrix3x3 matrixIstWerte;
    matrixIstWerte.setRotation(transf->getRotation());
    tf::Quaternion quat;
    matrixIstWerte.getRotation(quat);
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

    if(debugOut) {
        std::cout << "Current pose of " << Core::getLimbString(limb) << " is \n  "
                  << currentPose.roll/M_PI*180.0 << "° / " << currentPose.pitch/M_PI*180.0 << "° / " << currentPose.yaw/M_PI*180.0 << "°\n  "
                  << currentPose.x << "m / " << currentPose.y << "m / " << currentPose.z << "m" << std::endl;
    }

    return currentPose;
}

geometry_msgs::Pose Walk::transformToPlan(Core::Limb limb, Walk::pose pose)
{
    geometry_msgs::Pose targetPose;
    tf::StampedTransform* transf = core->getTF(limb);
    tf::Matrix3x3 matrixZumDraufaddieren, matrixIstWerte;
    // winkel addieren
    matrixZumDraufaddieren.setRPY(pose.pitch, pose.roll, pose.yaw);
    matrixIstWerte.setRotation(transf->getRotation());
    matrixZumDraufaddieren *= matrixIstWerte;
    tf::Quaternion quat;
    matrixZumDraufaddieren.getRotation(quat);
    targetPose.orientation.x = quat.getX();
    targetPose.orientation.y = quat.getY();
    targetPose.orientation.z = quat.getZ();
    targetPose.orientation.w = quat.getW();

    // translation addieren
    targetPose.position.x = static_cast<double>(transf->getOrigin().x()) + pose.x;
    targetPose.position.y = static_cast<double>(transf->getOrigin().y()) + pose.y;
    targetPose.position.z = static_cast<double>(transf->getOrigin().z()) + pose.z;

    // Debug ausgabe
    double roll, pitch, yaw;
    matrixZumDraufaddieren.getRPY(roll, pitch, yaw);
    std::cout << "Transformation:\n  RPY: " << roll << " / " << pitch << " / " << yaw << "\n  XYZ: " << targetPose.position.x << " / " << targetPose.position.y  << " / " << targetPose.position.z << std::endl;

    return targetPose;
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
