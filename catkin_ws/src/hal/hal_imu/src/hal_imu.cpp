#include "hal_imu.hpp"
#include "hal_imuInterfaces.hpp"

/* Services servers interface implementation */
ImuServersRos::ImuServersRos(ros::NodeHandle *node) : nodeHandle(node)
{
}

void ImuServersRos::advertiseGetHandleService(Imu *imu)
{
    imuGetHandleServerRos = nodeHandle->advertiseService("hal_imuGetHandle", &Imu::getHandle, imu);
}

/* IMU implementation */
Imu::Imu(ImuServers *imuServiceServers) : imuServers(imuServiceServers)
{
    imuServers->advertiseGetHandleService(this);
}

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

    ImuServersRos imuServiceServersRos(&node);

    Imu Imu(&imuServiceServersRos);

    ros::spin();

    return 0;
}