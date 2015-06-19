#include "limbpose.h"

LimbPose::operator geometry_msgs::Pose() const
{
    tf::Quaternion q;
    q.setRPY(default_roll, default_pitch, default_yaw);
    geometry_msgs::Pose gpose;
    gpose.orientation.x = q.getX();
    gpose.orientation.y = q.getY();
    gpose.orientation.z = q.getZ();
    gpose.orientation.w = q.getW();
    gpose.position.x = default_x;
    gpose.position.y = default_y;
    gpose.position.z = default_z;
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


LimbPose &LimbPose::setParameterMul(const std::string param_name, const double roll, const double pitch, const double yaw, const double x, const double y, const double z)
{
    params_mul[param_name] = std::vector<double> {roll, pitch, yaw, x, y, z};
    return *this;
}

LimbPose &LimbPose::setParameterMul(std::map<std::string, std::vector<double> > parameters)
{
    for( std::pair<std::string, std::vector<double> > param : parameters)
    {
        auto& v = param.second;
        if(v.size() != 6)
        {
            throw std::runtime_error("LimbPose::setParameterMul(): Wrong number of values for parameter \""+param.first+"\".");
        }
        setParameterMul(param.first, v[0], v[1], v[2], v[3], v[4], v[5]);
    }
}

LimbPose &LimbPose::setParameterAdd(const std::string param_name, const double roll, const double pitch, const double yaw, const double x, const double y, const double z)
{
    params_add[param_name] = std::vector<double> {roll, pitch, yaw, x, y, z};
    return *this;
}

LimbPose &LimbPose::setParameterAdd(std::map<std::string, std::vector<double> > parameters)
{
    for( std::pair<std::string, std::vector<double> > param : parameters)
    {
        auto& v = param.second;
        if(v.size() != 6)
        {
            throw std::runtime_error("LimbPose::setParameterAdd(): Wrong number of values for parameter \""+param.first+"\".");
        }
        setParameterAdd(param.first, v[0], v[1], v[2], v[3], v[4], v[5]);
    }
}

LimbPose &LimbPose::setParameterInput(const std::string param_name, const double input)
{
    if(inputs.count(param_name) == 0)
    {
        throw std::runtime_error("LimbPose::setParameterInput(): Parameter \"" + param_name + "\" does not exist.");
    }
    inputs[param_name] = input;
    return *this;
}

LimbPose &LimbPose::setParameterInputs(const std::map<std::string, double> inputs)
{
    for(std::pair<std::string, double> in : inputs)
    {
        setParameterInput(in.first, in.second);
    }
}

LimbPose LimbPose::operator+(const LimbPose &rlp) const
{

    if(this->limb != rlp.limb)
    {
        throw std::runtime_error("Cannot add the values of limbs "+Core::getLimbString(this->limb)+" and "+Core::getLimbString(rlp.limb)+", because they are different limbs...");
    }

    return LimbPose(
                this->limb,
                this->default_roll + rlp.default_roll,
                this->default_pitch + rlp.default_pitch,
                this->default_yaw + rlp.default_yaw,
                this->default_x + rlp.default_x,
                this->default_y + rlp.default_y,
                this->default_z + rlp.default_z
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
                this->default_roll - rlp.default_roll,
                this->default_pitch - rlp.default_pitch,
                this->default_yaw - rlp.default_yaw,
                this->default_x - rlp.default_x,
                this->default_y - rlp.default_y,
                this->default_z - rlp.default_z
                );
}

LimbPose& LimbPose::operator-=(const LimbPose &rlp)
{
    *this = *this - rlp;
    return *this;
}

double LimbPose::calcRoll(const std::map<std::string, double> new_inputs)
{
    setParameterInputs(new_inputs);
    return calcValue(0);
}

double LimbPose::calcPitch(const std::map<std::string, double> new_inputs)
{
    setParameterInputs(new_inputs);
    return calcValue(1);
}

double LimbPose::calcYaw(const std::map<std::string, double> new_inputs)
{
    setParameterInputs(new_inputs);
    return calcValue(2);
}

double LimbPose::calcX(const std::map<std::string, double> new_inputs)
{
    setParameterInputs(new_inputs);
    return calcValue(3);
}

double LimbPose::calcY(const std::map<std::string, double> new_inputs)
{
    setParameterInputs(new_inputs);
    return calcValue(4);
}
double LimbPose::calcZ(const std::map<std::string, double> new_inputs)
{
    setParameterInputs(new_inputs);
    return calcValue(5);
}

double LimbPose::calcValue(const int number) const
{
    double result;

    // load correct default value
    switch(number)
    {
    case 0:
        result = default_roll;
        break;
    case 1:
        result = default_pitch;
        break;
    case 2:
        result = default_yaw;
        break;
    case 3:
        result = default_x;
        break;
    case 4:
        result = default_y;
        break;
    case 5:
        result = default_z;
        break;
    default:
        throw std::runtime_error("LimbPose::calcValue(): unknown internal value number "+std::to_string(number)+".");
    }

    // perform "value = parameter*input * value" for every parameter
    for(std::pair<std::string, std::vector<double> > p_mul : params_mul)
    {
        const std::string param_name = p_mul.first;
        if(inputs.count(param_name) == 0)
        {
            throw std::runtime_error("No input value for parameter \""+param_name+"\" given.");
        }

        result *= p_mul.second.at(number) * inputs.at(param_name);
    }

    // perform "value = parameter*input + value" for every parameter
    for(std::pair<std::string, std::vector<double> > p_add : params_add)
    {
        const std::string param_name = p_add.first;
        if(inputs.count(param_name) == 0)
        {
            throw std::runtime_error("No input value for parameter \""+param_name+"\" given.");
        }

        result += p_add.second.at(number) * inputs.at(param_name);
    }

    return result;
}
