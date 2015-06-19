/*  HOly Walk walk.cpp
 *  A number of poses that the robot can move to.
 *  Laurenz & Simon & Carlo 2015-06-11
 */
#ifndef LIMBPOSE_H
#define LIMBPOSE_H

#include "../core.h"

class LimbPose {
public:
    LimbPose(): limb(Core::Limb::ERROR),  default_roll(0.0), default_pitch(0.0), default_yaw(0.0), default_x(0.0), default_y(0.0), default_z(0.0) {}
    // The standard way of constructing a LimbPose is to say what limb it refers to and state its orientation and position.
    LimbPose(const Core::Limb limb,
             const double roll,
             const double pitch,
             const double yaw,
             const double x,
             const double y,
             const double z) : limb(limb),  default_roll(roll), default_pitch(pitch), default_yaw(yaw), default_x(x), default_y(y), default_z(z) {}

    // Use the Core to construct a LimbPose that resembles the actual current Pose of a specific limb
    static LimbPose fromCurrentPose(const Core::Limb limb, Core &core);

    // LimbPoses can be parameterized. The elements of a LimbPose (roll, pitch, ...) are computed before use as "(MulParams * input * defaultValues) + AddParams * input".
    // The default value is set explicitly or from constructor
    // The MulParams and AddParams are set with the addParameterMul/addParameterAdd methods.
    LimbPose& setParameterMul(const std::string param_name, const double default_roll, const double default_pitch, const double default_yaw, const double default_x, const double default_y, const double default_z);
    LimbPose& setParameterMul(std::map<std::string, std::vector<double> > parameters);
    LimbPose& setParameterAdd(const std::string param_name, const double default_roll, const double default_pitch, const double default_yaw, const double default_x, const double default_y, const double default_z);
    LimbPose& setParameterAdd(std::map<std::string, std::vector<double> > parameters);

    // The input for every parameter is set with the following method
    LimbPose& setParameterInput(const std::string param_name, const double input);
    LimbPose& setParameterInputs(const std::map<std::string, double> inputs);

    // Adding / Subtracting two LimbPoses merges their values if they refer to the same Core::Limb, but throws a runtime_exception if they have different Core::Limbs.
    LimbPose operator+ (const LimbPose& rlp) const;
    LimbPose& operator+=(const LimbPose& rlp);
    LimbPose operator- (const LimbPose& rlp) const;
    LimbPose& operator-=(const LimbPose& rlp);

    // Provides a way to cast a LimbPose to an object of the ROS-compatible message type geometry_msgs::Pose
    operator geometry_msgs::Pose () const;

    // The limb this LimbPose is referring to
    Core::Limb limb;

    // Calculate output values for given (or last) parameter input values
    double calcRoll(const std::map<std::string, double> new_inputs = {});
    double calcPitch(const std::map<std::string, double> new_inputs = {});
    double calcYaw(const std::map<std::string, double> new_inputs = {});
    double calcX(const std::map<std::string, double> new_inputs = {});
    double calcY(const std::map<std::string, double> new_inputs = {});
    double calcZ(const std::map<std::string, double> new_inputs = {});
    double calcRoll() const;
    double calcPitch() const;
    double calcYaw() const;
    double calcX() const;
    double calcY() const;
    double calcZ() const;

    // The unparameterized / default pose of the limb
    double default_roll, default_pitch, default_yaw, default_x, default_y, default_z;

private:
    std::map<std::string, std::vector<double> > params_mul, params_add;
    std::map<std::string, double> inputs;

    double calcValue(const int number) const;

};

#endif // LIMBPOSE_H
