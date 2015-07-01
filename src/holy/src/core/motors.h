#ifndef MOTORS_H
#define MOTORS_H

#include <stdbool.h>
#include <string>

class Motors
{
private:
    Motors();

public:
    static bool setCompliance(bool setReset);
    static int runComplianceBash(std::string filename);

    const static std::string resetBashFilename;
    const static std::string setBashFilename;
};

#endif // MOTORS_H
