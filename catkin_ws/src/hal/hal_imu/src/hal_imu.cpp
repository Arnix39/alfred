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
    setClockSource();
    setSleepDisabled();
    writeDmp();
    enable6AxisQuaternion();
    enableDmp();
    //    calibrateAccelerometer();
    //    enableGyroCalibrationOnDMP();
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

void Imu::enableDmp(void)
{
    /* Disable FIFO for accelerometer and gyroscope */
    if (writeByteInRegister(MPU6050_FIFO_ENABLE_REGISTER, 0))
    {
        ROS_INFO("Sucessfully disabled FIFO for accelerometer and gyroscope.");
    }
    else
    {
        ROS_ERROR("Failed to disable FIFO for accelerometer and gyroscope!");
    }

    /* Enable interrupts for DMP */
    if (writeByteInRegister(MPU6050_INTERRUPT_ENABLE_REGISTER, MPU6050_DMP_INTERRUPT_ONLY))
    {
        ROS_INFO("Sucessfully enabled interrupts for DMP.");
    }
    else
    {
        ROS_ERROR("Failed to enable interrupts for DMP!");
    }
}

bool Imu::writeBitInRegister(uint8_t registerToWrite, uint8_t bitToWrite, uint8_t valueOfBit)
{
    hal_pigpio::hal_pigpioI2cReadByteData i2cReadByteDataSrv;
    hal_pigpio::hal_pigpioI2cWriteByteData i2cWriteByteDataSrv;
    uint8_t registerValue;
    uint8_t newRegisterValue;

    i2cReadByteDataSrv.request.handle = imuHandle;
    i2cReadByteDataSrv.request.deviceRegister = registerToWrite;

    imuClients->getReadByteDataClientHandle()->call(i2cReadByteDataSrv);

    if (i2cReadByteDataSrv.response.hasSucceeded)
    {
        registerValue = i2cReadByteDataSrv.response.value;
    }
    else
    {
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

    imuClients->getWriteByteDataClientHandle()->call(i2cWriteByteDataSrv);

    if (i2cWriteByteDataSrv.response.hasSucceeded)
    {
        return true;
    }
    else
    {
        return false;
    }

    return true;
}

void Imu::setClockSource(void)
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

void Imu::setSleepDisabled(void)
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

void Imu::calibrateAccelerometer(void)
{
}

void Imu::enable6AxisQuaternion(void)
{
    const unsigned char dmp6AxisQuaternionEnable[MPU6050_DMP_FEATURE_6X_LP_QUAT_SIZE] = {0x20, 0x28, 0x30, 0x38};

    if (writeDataToDmp(MPU6050_DMP_FEATURE_6X_LP_QUAT, MPU6050_DMP_FEATURE_6X_LP_QUAT_SIZE, dmp6AxisQuaternionEnable))
    {
        ROS_INFO("Sucessfully enabled 6 axis quaternions on DMP.");
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
        ROS_INFO("Sucessfully enabled gyroscope calibration on DMP.");
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
            ROS_ERROR("Failed to write data address MSB!");
            return false;
        }
    }
    else
    {
        ROS_ERROR("Failed to write data address LSB!");
        return false;
    }

    imuRegister = MPU6050_READ_WRITE_REGISTER;
    for (uint8_t index; index < size; ++index)
    {
        writeSuccess = writeByteInRegister(imuRegister, data[index]);
        if (!writeSuccess)
        {
            ROS_ERROR("Failed to write data!");
            break;
        }
    }

    return writeSuccess;
}

bool Imu::writeByteInRegister(uint8_t registerToWrite, uint8_t value)
{
    hal_pigpio::hal_pigpioI2cWriteByteData i2cWriteByteDataSrv;

    i2cWriteByteDataSrv.request.handle = imuHandle;
    i2cWriteByteDataSrv.request.deviceRegister = registerToWrite;
    i2cWriteByteDataSrv.request.value = value;

    imuClients->getWriteByteDataClientHandle()->call(i2cWriteByteDataSrv);

    if (i2cWriteByteDataSrv.response.hasSucceeded)
    {
        return true;
    }
    else
    {
        ROS_ERROR("Failed to write byte!");
        return false;
    }
}

int16_t Imu::readByteFromRegister(uint8_t registerToRead)
{
    hal_pigpio::hal_pigpioI2cReadByteData i2cReadByteDataSrv;

    i2cReadByteDataSrv.request.handle = imuHandle;
    i2cReadByteDataSrv.request.deviceRegister = registerToRead;

    imuClients->getReadByteDataClientHandle()->call(i2cReadByteDataSrv);

    if (i2cReadByteDataSrv.response.hasSucceeded)
    {
        return i2cReadByteDataSrv.response.value;
    }
    else
    {
        ROS_ERROR("Failed to read byte!");
        return -1;
    }
}

void Imu::publishMessage(void)
{
    hal_imu::hal_imuMsg message;

    message.angleInDeg = angle;
    imuPublisher->publish(message);
}

void Imu::readMpuData(void)
{
    int16_t valueRead;
    uint16_t fifoCount;
    std::vector<uint8_t> fifoData;

    /* Get number of bytes in the FIFO */
    valueRead = readByteFromRegister(MPU6050_FIFO_COUNT_H_REGISTER);
    if (valueRead >= 0)
    {
        fifoCount = ((uint8_t)valueRead) << 8;

        valueRead = readByteFromRegister(MPU6050_FIFO_COUNT_L_REGISTER);
        if (valueRead >= 0)
        {
            fifoCount |= ((uint8_t)valueRead);
            ROS_INFO("Successfully read the number of bytes in the FIFO.");
        }
        else
        {
            ROS_ERROR("Failed to read the number of bytes in the FIFO!");
        }
    }
    else
    {
        ROS_ERROR("Failed to read the number of bytes in the FIFO!");
    }

    if (fifoCount == MPU6050_DMP_FIFO_QUAT_SIZE)
    {
        for (int index = 0; index < MPU6050_DMP_FIFO_QUAT_SIZE; index++)
        {
            valueRead = readByteFromRegister(MPU6050_FIFO_REGISTER);

            if (valueRead >= 0)
            {
                fifoData.push_back((uint8_t)valueRead);
            }
            else
            {
                ROS_ERROR("Failed to read FIFO!");
                break;
            }
        }

        if (fifoData.size() == MPU6050_DMP_FIFO_QUAT_SIZE)
        {
            ROS_ERROR("Sucessfully read FIFO!");
        }
    }
    else
    {
        ROS_ERROR("Unexpected number of bytes in FIFO!");
    }
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
        imu.readMpuData();
        imu.publishMessage();

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}