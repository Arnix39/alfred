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

/* Services clients interface implementation */
ImuClientsRos::ImuClientsRos(ros::NodeHandle *node) : i2cReadByteDataClientRos(node->serviceClient<hal_pigpio::hal_pigpioI2cReadByteData>("hal_pigpioI2cReadByteData")),
                                                      i2cWriteByteDataClientRos(node->serviceClient<hal_pigpio::hal_pigpioI2cWriteByteData>("hal_pigpioI2cWriteByteData")),
                                                      i2cGetHandleClientRos(node->serviceClient<hal_imu::hal_imuGetHandle>("hal_imuGetHandle"))
{
}

ros::ServiceClient *ImuClientsRos::getReadByteDataClientHandle()
{
    return &i2cReadByteDataClientRos;
}

ros::ServiceClient *ImuClientsRos::getWriteByteDataClientHandle()
{
    return &i2cWriteByteDataClientRos;
}

ros::ServiceClient *ImuClientsRos::getGetHandleClientHandle()
{
    return &i2cGetHandleClientRos;
}

/* IMU implementation */
Imu::Imu(ImuPublisher *imuMessagePublisher, ImuClients *imuServiceClients) : imuPublisher(imuMessagePublisher),
                                                                             imuClients(imuServiceClients)
{
    hal_imu::hal_imuGetHandle i2cGetHandleSrv;
    imuServiceClients->getGetHandleClientHandle()->call(i2cGetHandleSrv);
    imuHandle = i2cGetHandleSrv.response.handle;
}

void Imu::init(void)
{
    this->writeDmp();
    // this->enable6AxisQuaternion();
    // this->calibrateAccelerometer();
    // this->enableGyroCalibrationOnDMP();
}

void Imu::writeDmp(void)
{
    hal_imu::hal_imuWriteDmpGoal imuDmpWritingGoal;
    imuActionClient_t imuDmpWritingClient("imuDMPWriting", true);
    ROS_INFO("Waiting for DMP writing server...");
    imuDmpWritingClient.waitForServer();
    ROS_INFO("Connected to DMP writing server.");
    imuDmpWritingGoal.write = true;
    ROS_INFO("Requesting DMP code writing...");
    imuDmpWritingClient.sendGoal(imuDmpWritingGoal);
    ROS_INFO("Request sent.");

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

    imuRegister = MPU6050_READ_WRITE_REGISTER;
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
    ImuClientsRos imuServiceClientsRos(&node);

    Imu imu(&imuMessagePublisherRos, &imuServiceClientsRos);
    imu.init();

    ros::Rate loop_rate(200);

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}