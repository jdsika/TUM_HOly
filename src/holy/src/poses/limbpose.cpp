#include "limbpose.h"

LimbPose::operator geometry_msgs::Pose() const
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

LimbPose LimbPose::fromCurrentPose(Core::Limb limb, Core &core)
{
    geometry_msgs::Pose currPos;
    tf::StampedTransform* transf = core.getTF(limb);
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

    double r,p,y;
    tf::Matrix3x3(tf::Quaternion(currPos.orientation.x,
                                 currPos.orientation.y,
                                 currPos.orientation.z,
                                 currPos.orientation.w)
                  ).getEulerYPR(y, p, r);

    return LimbPose(limb, r, p, y, currPos.position.x, currPos.position.y, currPos.position.z);
}


LimbPose LimbPose::operator+(const LimbPose &rlp) const
{

    if(this->limb != rlp.limb)
    {
        throw std::runtime_error("Cannot add the values of limbs "+Core::getLimbString(this->limb)+" and "+Core::getLimbString(rlp.limb)+", because they are different limbs...");
    }

    return LimbPose(
                this->limb,
                this->roll + rlp.roll,
                this->pitch + rlp.pitch,
                this->yaw + rlp.yaw,
                this->x + rlp.x,
                this->y + rlp.y,
                this->z + rlp.z
                );
}

LimbPose& LimbPose::operator+=(const LimbPose &rlp)
{
    *this = *this + rlp;
    return *this;
}

LimbPose LimbPose::operator-(const LimbPose &rlp) const
{
    if(this->limb != rlp.limb)
    {
        throw std::runtime_error("Cannot subtract the values of limbs "+Core::getLimbString(this->limb)+" and "+Core::getLimbString(rlp.limb)+", because they are different limbs...");
    }

    return LimbPose(
                this->limb,
                this->roll - rlp.roll,
                this->pitch - rlp.pitch,
                this->yaw - rlp.yaw,
                this->x - rlp.x,
                this->y - rlp.y,
                this->z - rlp.z
                );
}

LimbPose& LimbPose::operator-=(const LimbPose &rlp)
{
    *this = *this - rlp;
    return *this;
}
