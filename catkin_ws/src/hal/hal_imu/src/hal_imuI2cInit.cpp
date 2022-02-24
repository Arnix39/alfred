#include "hal_imuI2cInit.hpp"
#include "hal_imuI2cInitInterfaces.hpp"

/* Clients interface implementation */

imuServers->advertiseGetHandleService(this);

bool Imu::getHandle(hal_imu::hal_imuGetHandle::Request &req,
                    hal_imu::hal_imuGetHandle::Response &res)
{
    res.handle = imuHandle;
    return true;
}

i2cOpenClientRos(node->serviceClient<hal_pigpio::hal_pigpioI2cOpen>("hal_pigpioI2cOpen")),
    i2cCloseClientRos(node->serviceClient<hal_pigpio::hal_pigpioI2cClose>("hal_pigpioI2cClose")),

    ros::ServiceClient *ImuClientsRos::getI2cOpenHandle()
{
    return &i2cOpenClientRos;
}

ros::ServiceClient *ImuClientsRos::getI2cCloseHandle()
{
    return &i2cCloseClientRos;
}

/* Services servers interface implementation */
ImuServersRos::ImuServersRos(ros::NodeHandle *node) : nodeHandle(node)
{
}

void ImuServersRos::advertiseGetHandleService(Imu *imu)
{
    imuGetHandleServerRos = nodeHandle->advertiseService("hal_imuGetHandle", &Imu::getHandle, imu);
}

// destructor

hal_pigpio::hal_pigpioI2cClose i2cCloseSrv;
i2cCloseSrv.request.handle = imuHandle;
imuClients->getI2cCloseHandle()->call(i2cCloseSrv);

void Imu::initI2cCommunication(void)
{
    hal_pigpio::hal_pigpioI2cOpen i2cOpenSrv;

    i2cOpenSrv.request.bus = IMU_I2C_BUS;
    i2cOpenSrv.request.address = IMU_I2C_ADDRESS;

    imuClients->getI2cOpenHandle()->call(i2cOpenSrv);
    if (i2cOpenSrv.response.hasSucceeded)
    {
        imuHandle = i2cOpenSrv.response.handle;
    }
    else
    {
        ROS_ERROR("Unable to open I2C communication with device %u on bus %u!", i2cOpenSrv.request.address, i2cOpenSrv.request.bus);
    }
}

ImuServersRos imuServiceServersRos(&node);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hal_imuI2cInit");
    ros::NodeHandle node;

    ImuDmpWritingClientsRos imuServiceClients(&node);
    ImuDmpWritingActionServerRos imuWriteDmpServer(&node);

    ImuDmpWritingServer imuDmpWritingServer(&imuWriteDmpServer, &imuServiceClients);

    ros::spin();

    return 0;
}