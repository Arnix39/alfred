#include "hal_imu.hpp"

bool Imu::getHandle(hal_imu::hal_imuGetHandle::Request &req,
                    hal_imu::hal_imuGetHandle::Response &res)
{
    res.handle = imuHandle;
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hal_imu");
    ros::NodeHandle node;

    ros::spin();

    return 0;
}