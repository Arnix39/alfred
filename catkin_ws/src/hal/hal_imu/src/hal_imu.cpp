#include "hal_imu.hpp"
#include "hal_imuInterfaces.hpp"
#include "hal_imuMPU6050.hpp"

/* Publisher interface implementation */
ImuPublisherRos::ImuPublisherRos(ros::NodeHandle *node) : imuPublisherRos(node->advertise<hal_imu::hal_imuMsg>("angleValue", 1000))
{
}

void ImuPublisherRos::publish(hal_imu::hal_imuMsg message)
{
    imuPublisherRos.publish(message);
}

/* Services servers interface implementation */
ImuServersRos::ImuServersRos(ros::NodeHandle *node) : nodeHandle(node)
{
}

void ImuServersRos::advertiseGetHandleService(Imu *imu)
{
    imuGetHandleServerRos = nodeHandle->advertiseService("hal_imuGetHandle", &Imu::getHandle, imu);
}

/* Services clients interface implementation */
ImuClientsRos::ImuClientsRos(ros::NodeHandle *node) : i2cOpenClientRos(node->serviceClient<hal_pigpio::hal_pigpioI2cOpen>("hal_pigpioI2cOpen")),
                                                      i2cCloseClientRos(node->serviceClient<hal_pigpio::hal_pigpioI2cClose>("hal_pigpioI2cClose")),
                                                      i2cReadByteDataClientRos(node->serviceClient<hal_pigpio::hal_pigpioI2cReadByteData>("hal_pigpioI2cReadByteData")),
                                                      i2cWriteByteDataClientRos(node->serviceClient<hal_pigpio::hal_pigpioI2cWriteByteData>("hal_pigpioI2cWriteByteData"))
{
}

ros::ServiceClient *ImuClientsRos::getI2cOpenHandle()
{
    return &i2cOpenClientRos;
}

ros::ServiceClient *ImuClientsRos::getI2cCloseHandle()
{
    return &i2cCloseClientRos;
}

ros::ServiceClient *ImuClientsRos::getReadByteDataClientHandle()
{
    return &i2cReadByteDataClientRos;
}

ros::ServiceClient *ImuClientsRos::getWriteByteDataClientHandle()
{
    return &i2cWriteByteDataClientRos;
}

/* IMU implementation */
Imu::Imu(ImuPublisher *imuMessagePublisher, ImuServers *imuServiceServers, ImuClients *imuServiceClients) : imuPublisher(imuMessagePublisher),
                                                                                                            imuServers(imuServiceServers),
                                                                                                            imuClients(imuServiceClients)
{
    imuServers->advertiseGetHandleService(this);
}

Imu::~Imu()
{
    hal_pigpio::hal_pigpioI2cClose i2cCloseSrv;

    i2cCloseSrv.request.handle = imuHandle;

    imuClients->getI2cCloseHandle()->call(i2cCloseSrv);
}

bool Imu::getHandle(hal_imu::hal_imuGetHandle::Request &req,
                    hal_imu::hal_imuGetHandle::Response &res)
{
    res.handle = imuHandle;
    return true;
}

void Imu::init(void)
{
    this->initI2cCommunication();
    this->writeDmp();
    this->enable6AxisQuaternion();
    this->calibrateAccelerometer();
    this->enableGyroCalibrationOnDMP();
}

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

void Imu::writeDmp(void)
{
    hal_imu::hal_imuWriteDmpGoal imuDmpWritingGoal;
    imuActionClient_t imuDmpWritingClient("imuDMPWriting", false);
    imuDmpWritingClient.waitForServer();

    imuDmpWritingGoal.write = true;
    imuDmpWritingClient.sendGoal(imuDmpWritingGoal);

    imuDmpWritingClient.waitForResult();
    if (imuDmpWritingClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("DMP code written successfully!");
    }
    else
    {
        ROS_ERROR("Error while writing DMP code!");
    }
}

void Imu::calibrateAccelerometer(void)
{
}

void Imu::enable6AxisQuaternion(void)
{
    const unsigned char dmp6AxisQuaternionEnable[MPU6050_DMP_FEATURE_6X_LP_QUAT_SIZE] = {0x20, 0x28, 0x30, 0x38};

    if (writeDataToDmp(MPU6050_DMP_FEATURE_6X_LP_QUAT, MPU6050_DMP_FEATURE_6X_LP_QUAT_SIZE, dmp6AxisQuaternionEnable))
    {
        ROS_INFO("Sucessfully enabled 6 axis quaternions on DMP!");
    }
    else
    {
        ROS_ERROR("Error while enabling 6 axis quaternions on DMP!");
    }
}

void Imu::enableGyroCalibrationOnDMP(void)
{
    const unsigned char gyroscopeCalibrationEnable[MPU6050_DMP_FEATURE_GYRO_CAL_SIZE] = {0xb8, 0xaa, 0xb3, 0x8d, 0xb4, 0x98, 0x0d, 0x35, 0x5d};

    if (writeDataToDmp(MPU6050_CFG_MOTION_BIAS, MPU6050_DMP_FEATURE_GYRO_CAL_SIZE, gyroscopeCalibrationEnable))
    {
        ROS_INFO("Sucessfully enabled gyroscope calibration on DMP!");
    }
    else
    {
        ROS_ERROR("Error while enabling gyroscope calibration on DMP!");
    }
}

bool Imu::writeDataToDmp(uint16_t address, uint8_t size, const unsigned char *data)
{
    bool writeSuccess = false;

    uint8_t imuRegister = MPU6050_BANK_SELECTION_REGISTER;
    const uint8_t lsbAddress = (uint8_t)(address >> 8);
    const uint8_t msbAddress = (uint8_t)(address & 0xFF);
    writeSuccess = writeByteInRegister(imuRegister, lsbAddress);
    if (writeSuccess)
    {
        writeSuccess = writeByteInRegister(imuRegister, msbAddress);
        if (!writeSuccess)
        {
            return false;
        }
    }
    else
    {
        return false;
    }

    uint8_t imuRegister = MPU6050_READ_WRITE_REGISTER;
    for (uint8_t index; index < size; ++index)
    {
        writeSuccess = writeByteInRegister(imuRegister, data[index]);
        if (!writeSuccess)
        {
            break;
        }
    }

    return writeSuccess;
}

bool Imu::writeByteInRegister(uint8_t chipRegister, uint8_t value)
{
    hal_pigpio::hal_pigpioI2cWriteByteData i2cWriteByteDataSrv;
    hal_pigpio::hal_pigpioI2cReadByteData i2cReadByteDataSrv;

    i2cWriteByteDataSrv.request.handle = imuHandle;
    i2cReadByteDataSrv.request.handle = imuHandle;

    i2cWriteByteDataSrv.request.deviceRegister = chipRegister;
    i2cWriteByteDataSrv.request.value = value;
    imuClients->getWriteByteDataClientHandle()->call(i2cWriteByteDataSrv);

    if (i2cWriteByteDataSrv.response.hasSucceeded)
    {
        imuClients->getReadByteDataClientHandle()->call(i2cReadByteDataSrv);
        if (i2cReadByteDataSrv.response.hasSucceeded && (i2cReadByteDataSrv.response.value == value))
        {
            return true;
        }
    }

    return false;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hal_imu");
    ros::NodeHandle node;

    ImuPublisherRos imuMessagePublisherRos(&node);
    ImuServersRos imuServiceServersRos(&node);
    ImuClientsRos imuServiceClientsRos(&node);

    Imu imu(&imuMessagePublisherRos, &imuServiceServersRos, &imuServiceClientsRos);
    imu.init();

    ros::Rate loop_rate(200);

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}