#include "hal_imuI2cInit.hpp"
#include "hal_imuI2cInitInterfaces.hpp"

/* Clients interface implementation */
ImuI2cInitClientsRos::ImuI2cInitClientsRos(ros::NodeHandle *node) : i2cOpenClientRos(node->serviceClient<hal_pigpio::hal_pigpioI2cOpen>("hal_pigpioI2cOpen")),
                                                                    i2cCloseClientRos(node->serviceClient<hal_pigpio::hal_pigpioI2cClose>("hal_pigpioI2cClose"))
{
}

ros::ServiceClient *ImuI2cInitClientsRos::getI2cOpenHandle()
{
    return &i2cOpenClientRos;
}

ros::ServiceClient *ImuI2cInitClientsRos::getI2cCloseHandle()
{
    return &i2cCloseClientRos;
}

/* Servers interface implementation */
ImuI2cInitServersRos::ImuI2cInitServersRos(ros::NodeHandle *node) : nodeHandle(node)
{
}

void ImuI2cInitServersRos::advertiseGetHandleService(ImuI2cInit *imuI2cInit)
{
    imuGetHandleServerRos = nodeHandle->advertiseService("hal_imuGetHandle", &ImuI2cInit::getHandle, imuI2cInit);
}

/* Imu I2c Init implementation */
ImuI2cInit::ImuI2cInit(ImuI2cInitClients *imuI2cInitServiceClients, ImuI2cInitServers *imuI2cInitServiceServers) : imuI2cInitClients(imuI2cInitServiceClients),
                                                                                                                   imuI2cInitServers(imuI2cInitServiceServers)
{
    imuI2cInitServiceServers->advertiseGetHandleService(this);
}

ImuI2cInit::~ImuI2cInit()
{
    hal_pigpio::hal_pigpioI2cClose i2cCloseSrv;
    i2cCloseSrv.request.handle = imuHandle;
    imuI2cInitClients->getI2cCloseHandle()->call(i2cCloseSrv);
}

bool ImuI2cInit::getHandle(hal_imu::hal_imuGetHandle::Request &req,
                           hal_imu::hal_imuGetHandle::Response &res)
{
    res.handle = imuHandle;
    return true;
}

void ImuI2cInit::initI2cCommunication(void)
{
    hal_pigpio::hal_pigpioI2cOpen i2cOpenSrv;

    i2cOpenSrv.request.bus = IMU_I2C_BUS;
    i2cOpenSrv.request.address = MPU6050_I2C_ADDRESS;

    imuI2cInitClients->getI2cOpenHandle()->call(i2cOpenSrv);
    if (i2cOpenSrv.response.hasSucceeded)
    {
        imuHandle = i2cOpenSrv.response.handle;
    }
    else
    {
        ROS_ERROR("Unable to open I2C communication with device %u on bus %u!", i2cOpenSrv.request.address, i2cOpenSrv.request.bus);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hal_imuI2cInit");
    ros::NodeHandle node;

    ImuI2cInitClientsRos imuI2cInitServiceClients(&node);
    ImuI2cInitServersRos imuI2cInitServer(&node);

    ImuI2cInit imuI2cInit(&imuI2cInitServiceClients, &imuI2cInitServer);
    imuI2cInit.initI2cCommunication();

    ros::spin();

    return 0;
}