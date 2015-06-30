#include "motors.h"

#include <ros/service.h>    // this is used to set the compliance slope, margin and punch
#include <vector>

const std::string Motors::setBashFilename = "setcompliance_stairs.sh";
const std::string Motors::resetBashFilename = "resetcompliance_stairs.sh";


bool Motors::setCompliance()
{
    std::vector<std::string> side = {"/L_","/R_"};
    std::vector<std::string> joint = {"HAA", "HR", "HFE", "KFE", "AFE", "AR"};
    std::vector<std::string> service = {"_controller/set_speed",
                           "_controller/torque_enable",
                          "_controller/set_compliance_slope",
                          "_controller/set_compliance_margin",
                          "_controller/set_compliance_punch"
                          };

    std::vector<bool> answers;

    answers.resize(side.size()*joint.size()*service.size());

    std::vector<double> values = {5.3, 1.0, 12.0, 0.0, 0.0};

    for( std::string s : side) {
        for( std::string m : joint) {
            for(int i = 0; i < values.size(); ++i)
            {
                //ros::service::call(s+m+service.at(i), values[i], answers[i]);
                std::cout << s+m+service.at(i) << std::endl;
            }
        }
    }

    return true;
}

bool Motors::runComplianceBash(std::string filename)
{

}

Motors::Motors()
{
}
