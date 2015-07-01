#ifndef MOTORS_H
#define MOTORS_H

#include <stdbool.h>
#include <string>

class Helper
{
private:
    Helper();

public:
    static bool setCompliance(bool setReset);
    static int runComplianceBash(std::string filename);
    static std::string getPackagePath(std::string libname);
    static int setWorkingDirectory(const std::string path);

    const static std::string resetBashFilename;
    const static std::string setBashFilename;
};

#endif // MOTORS_H
