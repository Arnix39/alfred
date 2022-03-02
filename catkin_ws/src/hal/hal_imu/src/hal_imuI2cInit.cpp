#include "hal_imuI2cInit.hpp"
#include "hal_imuI2cInitInterfaces.hpp"

/* Clients interface implementation */
ImuI2cInitClientsRos::ImuI2cInitClientsRos(ros::NodeHandle *node) : i2cOpenClientRos(node->serviceClient<hal_pigpio::hal_pigpioI2cOpen>("hal_pigpioI2cOpen")),
                                                                    i2cCloseClientRos(node->serviceClient<hal_pigpio::hal_pigpioI2cClose>("hal_pigpioI2cClose")),
                                                                    i2cReadByteDataClientRos(node->serviceClient<hal_pigpio::hal_pigpioI2cReadByteData>("hal_pigpioI2cReadByteData")),
                                                                    i2cWriteByteDataClientRos(node->serviceClient<hal_pigpio::hal_pigpioI2cWriteByteData>("hal_pigpioI2cWriteByteData"))
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

ros::ServiceClient *ImuI2cInitClientsRos::getI2cReadByteDataClientHandle()
{
    return &i2cReadByteDataClientRos;
}

ros::ServiceClient *ImuI2cInitClientsRos::getI2cWriteByteDataClientHandle()
{
    return &i2cWriteByteDataClientRos;
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
        ROS_INFO("Handle %u received for communication with device %u on bus %u.", imuHandle, i2cOpenSrv.request.address, i2cOpenSrv.request.bus);
    }
    else
    {
        ROS_ERROR("Unable to receive handle for communication with device %u on bus %u!", i2cOpenSrv.request.address, i2cOpenSrv.request.bus);
    }
}

bool ImuI2cInit::writeBitInRegister(uint8_t registerToWrite, uint8_t bitToWrite, uint8_t valueOfBit)
{
    hal_pigpio::hal_pigpioI2cReadByteData i2cReadByteDataSrv;
    hal_pigpio::hal_pigpioI2cWriteByteData i2cWriteByteDataSrv;
    uint8_t registerValue;
    uint8_t newRegisterValue;

    i2cReadByteDataSrv.request.handle = imuHandle;
    i2cReadByteDataSrv.request.deviceRegister = registerToWrite;

    imuI2cInitClients->getI2cReadByteDataClientHandle()->call(i2cReadByteDataSrv);

    if (i2cReadByteDataSrv.response.hasSucceeded)
    {
        registerValue = i2cReadByteDataSrv.response.value;
        ROS_INFO("Read value (%u) from register %u on device with handle %u.", registerValue, i2cReadByteDataSrv.request.deviceRegister, i2cReadByteDataSrv.request.handle);
    }
    else
    {
        ROS_ERROR("Unable to read register %u on device with handle %u", i2cReadByteDataSrv.request.deviceRegister, i2cReadByteDataSrv.request.handle);
        return false;
    }

    if (valueOfBit == 1)
    {
        newRegisterValue = registerValue | (1 << bitToWrite);
    }
    else
    {
        newRegisterValue = registerValue & ~(1 << bitToWrite);
    }

    i2cWriteByteDataSrv.request.handle = imuHandle;
    i2cWriteByteDataSrv.request.deviceRegister = registerToWrite;
    i2cWriteByteDataSrv.request.value = newRegisterValue;

    imuI2cInitClients->getI2cWriteByteDataClientHandle()->call(i2cWriteByteDataSrv);

    if (i2cWriteByteDataSrv.response.hasSucceeded)
    {
        ROS_INFO("Wrote value (%u) on register %u on device with handle %u.", i2cWriteByteDataSrv.request.value, i2cWriteByteDataSrv.request.deviceRegister, i2cWriteByteDataSrv.request.handle);
    }
    else
    {
        ROS_ERROR("Unable to write value (%u) on register %u on device with handle %u", i2cWriteByteDataSrv.request.value, i2cWriteByteDataSrv.request.deviceRegister, i2cWriteByteDataSrv.request.handle);
        return false;
    }

    return true;
}

void ImuI2cInit::setClockSource(void)
{
    bool writeSuccess;

    writeSuccess = writeBitInRegister(MPU6050_POWER_MANAGEMENT_1_REGISTER, MPU6050_CLOCK_SOURCE_BIT_1, 1) && writeBitInRegister(MPU6050_POWER_MANAGEMENT_1_REGISTER, MPU6050_CLOCK_SOURCE_BIT_2, 0) && writeBitInRegister(MPU6050_POWER_MANAGEMENT_1_REGISTER, MPU6050_CLOCK_SOURCE_BIT_3, 0);

    if (writeSuccess)
    {
        ROS_INFO("Successfully enabled PLL_X clock source on device with handle %u.", imuHandle);
    }
    else
    {
        ROS_ERROR("Unable to enable PLL_X clock source on device with handle %u.", imuHandle);
    }
}

void ImuI2cInit::setSleepDisabled(void)
{
    if (writeBitInRegister(MPU6050_POWER_MANAGEMENT_1_REGISTER, MPU6050_ENABLE_SLEEP_BIT, 1))
    {
        ROS_INFO("Successfully disabled sleep mode on device with handle %u.", imuHandle);
    }
    else
    {
        ROS_ERROR("Unable to disable sleep mode on device with handle %u.", imuHandle);
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
    imuI2cInit.setClockSource();
    imuI2cInit.setSleepDisabled();

    ros::spin();

    return 0;
}