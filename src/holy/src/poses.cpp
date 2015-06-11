#include "poses.h"

const geometry_msgs::Pose pose::toGeoPose() const
{

    if(relative)
    {
        //
    }
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

void pose::print(bool progformat) const
{
    if(progformat)
    {
        // Walk::pose {0, 0, 0, 0, 0, 0}
        std::cout << "Pose contains:\n  Walk::pose {" << roll << ",  "  << pitch << ",  "  << yaw << ",  " << x << ", " << y << ", " << z << "}\n";
    } else {
        std::cout << "Pose contains:\n  " << roll << "° / "  << pitch << "° / "  << yaw << "\n  " << x << " / " << y << " / " << z << "\n";
    }
}
