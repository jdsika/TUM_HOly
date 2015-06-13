#include "poses.h"
#include "core.h"

#include <exception> // runtime_exception
#include <utility> // swap

RoboPose::RoboPose(const std::vector<LimbPose> limbs, const std::string objname)
    : objname(objname)
{
    this->limbs.reserve(limbs.size());
    for(const LimbPose &new_lp : limbs)
    {
        bool already_contained = false;
        for(const LimbPose &old_lp : this->limbs)
        {
            if(new_lp.limb == old_lp.limb)
            {
                already_contained = true;
                std::cerr << "RoboPose \""<<objname<<"\": The pose of limb " << Core::getLimbString(new_lp.limb) << " is already declared." << std::endl;
            }
        }
        if(!already_contained) {
           this->limbs.push_back(new_lp);
       }
    }
    std::cout << "RoboPose \""<<objname<<"\" now contains "<<this->limbs.size()<<" limbs"<<std::endl;
}

RoboPose RoboPose::fromCurrentPose(Core &core)
{
    return RoboPose(std::vector<LimbPose> {
                        LimbPose::fromCurrentPose(Core::Limb::LEFT_FOOT, core),
                        LimbPose::fromCurrentPose(Core::Limb::RIGHT_FOOT, core),
                        LimbPose::fromCurrentPose(Core::Limb::LEFT_HAND, core),
                        LimbPose::fromCurrentPose(Core::Limb::RIGHT_HAND, core),
                    });
}

RoboPose &RoboPose::setLimb(LimbPose limbPose)
{
    getLimb(limbPose.limb) = limbPose;

    return *this;
}

LimbPose& RoboPose::getLimb(Core::Limb limb)
{
    for (int i=0; i<limbs.size(); ++i)
    {
        if(limbs.at(i).limb == limb)
        {
            return limbs[i];
        }
    }
    throw std::runtime_error("RoboPose \""+objname+"\" does not contain a LimbPose for limb "+Core::getLimbString(limb)+".");
}

std::vector<LimbPose> &RoboPose::getLimbs()
{
    return limbs;
}

const std::vector<LimbPose> RoboPose::getLimbs() const
{
    const std::vector<LimbPose> ret = limbs;
    return ret;
}

const RoboPose &RoboPose::printInfo() const
{
    printInfoImpl();
    return *this;
}

RoboPose &RoboPose::printInfo()
{
    printInfoImpl();
    return *this;
}

void RoboPose::printInfoImpl() const
{
    std::cout << "RoboPose \""<<objname<<"\" contains "<<limbs.size()<<" limbs:\n";
    for(const LimbPose & lp : limbs)
    {
        std::cout << " - " << Core::getLimbString(lp.limb) << ": \n"
                  << "   " << r2d(lp.roll) << "° / " << r2d(lp.pitch) << "° / " << r2d(lp.yaw) << "°\n"
                  << "   " << lp.x << "m / " << lp.y << "m / " << lp.z << "m\n";
    }
    std::flush(std::cout);
}


RoboPose RoboPose::operator+(const RoboPose &rrp) const
{
    std::vector<LimbPose> result_limbs;

    // Add All Limbs of the left hand RoboPose to the result
    for(LimbPose const &lh_lp : this->limbs)
    {
        result_limbs.push_back(lh_lp);
    }

    // Go through the Limbs of the right hand RoboPose
    for(LimbPose const &rh_lp : rrp.limbs)
    {
        bool lh_lp_was_found = false;
        for(LimbPose &res_lp : result_limbs)
        {
            // If a Limb existed in the left hand RoboPose (and thus is in the result), add the two LimbPoses
            if(rh_lp.limb == res_lp.limb)
            {
                lh_lp_was_found = true;
                res_lp += rh_lp;
            }
        }
        // If the Limb was not found in the result (left hand RoboPose), include it in the result-RoboPose
        if(!lh_lp_was_found)
        {
            result_limbs.push_back(rh_lp);
        }
     }

    return RoboPose(result_limbs, this->objname+"+"+rrp.objname);
}

RoboPose& RoboPose::operator+=(const RoboPose &rrp)
{
    *this = *this + rrp;
    return *this;
}

RoboPose RoboPose::operator-(const RoboPose &rrp) const
{
    std::vector<LimbPose> result_limbs;

    // Add All Limbs of the left hand RoboPose to the result
    for(LimbPose const &lh_lp : this->limbs)
    {
        result_limbs.push_back(lh_lp);
    }

    // Go through the Limbs of the right hand RoboPose
    for(LimbPose const &rh_lp : rrp.limbs)
    {
        bool lh_lp_was_found = false;
        for(LimbPose &res_lp : result_limbs)
        {
            // If a Limb existed in the left hand RoboPose (and thus is in the result), subtract the two LimbPoses
            if(rh_lp.limb == res_lp.limb)
            {
                lh_lp_was_found = true;
                res_lp -= rh_lp;
            }
        }
        // If the Limb was not found in the result (left hand RoboPose), include it in the result-RoboPose
        if(!lh_lp_was_found)
        {
            result_limbs.push_back(rh_lp);
        }
     }

    return RoboPose(result_limbs);
}

RoboPose& RoboPose::operator-=(const RoboPose &rrp)
{
    *this = *this - rrp;
    return *this;
}

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
