#include "motors.h"

#include <ros/service.h>    // this is used to set the compliance slope, margin and punch
#include <vector>
#include <stdlib.h>
//#include <iostream>

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

int Motors::runComplianceBash(std::string filename)
{
    int i;
    std::cout << "Checking if processor is available..." << std::endl;
    if (system(NULL)) {
        std::cout << "Ok" << std::endl;
    }
    else {
        std::cout << "Failed" << std::endl;
        return 0;
    }
    std::cout << "Executing command DIR..." << std::endl;
    std::cout << "Name: " << filename.c_str() << std::endl;

    i = system(filename.c_str());

    std::cout << "The value returned was: " << i << std::endl;

    return i;
}

Motors::Motors()
{
}
