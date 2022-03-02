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
                                                      i2cReadWordDataClientRos(node->serviceClient<hal_pigpio::hal_pigpioI2cReadWordData>("hal_pigpioI2cReadWordData")),
                                                      i2cWriteWordDataClientRos(node->serviceClient<hal_pigpio::hal_pigpioI2cWriteWordData>("hal_pigpioI2cWriteWordData")),
                                                      i2cWriteBlockDataClientRos(node->serviceClient<hal_pigpio::hal_pigpioI2cWriteBlockData>("hal_pigpioI2cWriteBlockData")),
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

ros::ServiceClient *ImuClientsRos::getReadWordDataClientHandle()
{
    return &i2cReadWordDataClientRos;
}

ros::ServiceClient *ImuClientsRos::getWriteWordDataClientHandle()
{
    return &i2cWriteWordDataClientRos;
}

ros::ServiceClient *ImuClientsRos::getWriteBlockDataClientHandle()
{
    return &i2cWriteBlockDataClientRos;
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
    resetImu();
    setClockSource();
    setSleepDisabled();
    writeDmp();
    writeOrientationMatrix();
    enable6AxisQuaternion();
    enableDmp();
    //    calibrateAccelerometer();
    //    enableGyroCalibrationOnDMP();
}

void Imu::resetImu(void)
{
    bool writeSuccess;

    writeSuccess = writeBitInRegister(MPU6050_POWER_MANAGEMENT_1_REGISTER, MPU6050_RESET_BIT, 1);
    if (writeSuccess)
    {
        ROS_INFO("IMU resetting...");
        ros::Duration(0.1).sleep();

        writeSuccess = writeByteInRegister(MPU6050_POWER_MANAGEMENT_1_REGISTER, 0x00);
        if (writeSuccess)
        {
            ROS_INFO("Successfully resetted IMU.");
        }
        else
        {
            ROS_ERROR("Failed to reset IMU.");
        }
    }
    else
    {
        ROS_ERROR("Failed to reset IMU.");
    }
}

void Imu::setClockSource(void)
{
    bool writeSuccess;

    writeSuccess = writeBitInRegister(MPU6050_POWER_MANAGEMENT_1_REGISTER, MPU6050_CLOCK_SOURCE_BIT_1, 1) && writeBitInRegister(MPU6050_POWER_MANAGEMENT_1_REGISTER, MPU6050_CLOCK_SOURCE_BIT_2, 0) && writeBitInRegister(MPU6050_POWER_MANAGEMENT_1_REGISTER, MPU6050_CLOCK_SOURCE_BIT_3, 0);

    if (writeSuccess)
    {
        ROS_INFO("Successfully enabled PLL_X clock source.");
    }
    else
    {
        ROS_ERROR("Failed to enable PLL_X clock source.");
    }
}

void Imu::setSleepDisabled(void)
{
    if (writeBitInRegister(MPU6050_POWER_MANAGEMENT_1_REGISTER, MPU6050_ENABLE_SLEEP_BIT, 0))
    {
        ROS_INFO("Successfully disabled sleep mode.");
    }
    else
    {
        ROS_ERROR("Failed to disable sleep mode.");
    }
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

void Imu::writeOrientationMatrix(void)
{
    std::vector<uint8_t> dmpOrientationMatrixGyroAxes{0x4C, 0xCD, 0x6C};
    std::vector<uint8_t> dmpOrientationMatrixGyroSigns{0x36, 0x56, 0x76};
    std::vector<uint8_t> dmpOrientationMatrixAccelAxes{0x0C, 0xC9, 0x2C};
    std::vector<uint8_t> dmpOrientationMatrixAccelSigns{0x26, 0x46, 0x66};

    bool writeSuccess = true;

    if (!writeDataToDmp(MPU6050_DMP_ORIENTATION_MATRIX_GYRO_AXES, dmpOrientationMatrixGyroAxes))
    {
        writeSuccess = false;
        ROS_ERROR("Failed to write the gyroscope axes of the orientation matrix!");
    }

    if (!writeDataToDmp(MPU6050_DMP_ORIENTATION_MATRIX_GYRO_SIGNS, dmpOrientationMatrixGyroSigns))
    {
        writeSuccess = false;
        ROS_ERROR("Failed to write the gyroscope signs of the orientation matrix!");
    }

    if (!writeDataToDmp(MPU6050_DMP_ORIENTATION_MATRIX_ACCEL_AXES, dmpOrientationMatrixAccelAxes))
    {
        writeSuccess = false;
        ROS_ERROR("Failed to write the accelerometer axes of the orientation matrix!");
    }

    if (!writeDataToDmp(MPU6050_DMP_ORIENTATION_MATRIX_ACCEL_SIGNS, dmpOrientationMatrixAccelSigns))
    {
        writeSuccess = false;
        ROS_ERROR("Failed to write the accelerometer signs of the orientation matrix!");
    }

    if (writeSuccess)
    {
        ROS_INFO("Successfully wrote orientation matrix.");
    }
}

void Imu::enableDmp(void)
{
    bool enablingHasSucceeded = true;

    /* Disable interrupts */
    if (!writeByteInRegister(MPU6050_INTERRUPT_ENABLE_REGISTER, 0x00))
    {
        enablingHasSucceeded = false;
        ROS_ERROR("Failed to disable interrupts!");
    }

    /* Disable FIFO for accelerometer and gyroscope */
    if (!writeByteInRegister(MPU6050_FIFO_ENABLE_REGISTER, 0x00))
    {
        enablingHasSucceeded = false;
        ROS_ERROR("Failed to disable FIFO for accelerometer and gyroscope!");
    }

    /* Set user control register to 0 */
    if (!writeByteInRegister(MPU6050_USER_CONTROL_REGISTER, 0x00))
    {
        enablingHasSucceeded = false;
        ROS_ERROR("Failed to set user control register to zero!");
    }

    /* Reset DMP and FIFO */
    if (!writeByteInRegister(MPU6050_USER_CONTROL_REGISTER, (MPU6050_DMP_RESET_BIT | MPU6050_FIFO_RESET_BIT)))
    {
        enablingHasSucceeded = false;
        ROS_ERROR("Failed to reset DMP and FIFO!");
    }

    ros::Duration(0.1).sleep();

    /* Enable DMP and FIFO */
    if (!writeByteInRegister(MPU6050_USER_CONTROL_REGISTER, (MPU6050_DMP_EMABLE_BIT | MPU6050_FIFO_ENABLE_BIT)))
    {
        enablingHasSucceeded = false;
        ROS_ERROR("Failed to enable DMP and FIFO!");
    }

    /* Enable interrupts for DMP */
    if (!writeByteInRegister(MPU6050_INTERRUPT_ENABLE_REGISTER, MPU6050_DMP_INTERRUPT_ONLY))
    {
        enablingHasSucceeded = false;
        ROS_ERROR("Failed to enable interrupts for DMP!");
    }

    /* Disable FIFO for accelerometer and gyroscope */
    if (!writeByteInRegister(MPU6050_FIFO_ENABLE_REGISTER, 0x00))
    {
        enablingHasSucceeded = false;
        ROS_ERROR("Failed to disable FIFO for accelerometer and gyroscope!");
    }

    if (enablingHasSucceeded)
    {
        ROS_INFO("Successfully enabled DMP.");
    }
}

void Imu::calibrateAccelerometer(void)
{
}

void Imu::enable6AxisQuaternion(void)
{
    std::vector<uint8_t> dmp6AxisQuaternionEnable{0x20, 0x28, 0x30, 0x38};

    if (writeDataToDmp(MPU6050_DMP_FEATURE_6X_LP_QUAT, dmp6AxisQuaternionEnable))
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
    std::vector<uint8_t> gyroscopeCalibrationEnable{0xb8, 0xaa, 0xb3, 0x8d, 0xb4, 0x98, 0x0d, 0x35, 0x5d};

    if (writeDataToDmp(MPU6050_CFG_MOTION_BIAS, gyroscopeCalibrationEnable))
    {
        ROS_INFO("Sucessfully enabled gyroscope calibration on DMP.");
    }
    else
    {
        ROS_ERROR("Error while enabling gyroscope calibration on DMP!");
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

bool Imu::writeDataToDmp(uint16_t address, std::vector<uint8_t> data)
{
    bool writeSuccess = false;

    uint8_t imuRegister = MPU6050_BANK_SELECTION_REGISTER;
    writeSuccess = writeWordInRegister(imuRegister, address);
    if (!writeSuccess)
    {
        ROS_ERROR("Failed to write address!");
        return false;
    }
    /*const uint8_t msbAddress = (uint8_t)(address >> 8);
    const uint8_t lsbAddress = (uint8_t)(address & 0xFF);
    writeSuccess = writeByteInRegister(imuRegister, msbAddress);
    if (writeSuccess)
    {
        writeSuccess = writeByteInRegister(imuRegister, lsbAddress);
        if (!writeSuccess)
        {
            ROS_ERROR("Failed to write data address LSB!");
            return false;
        }
    }
    else
    {
        ROS_ERROR("Failed to write data address MSB!");
        return false;
    }*/

    writeDataBlock(MPU6050_READ_WRITE_REGISTER, data);
    /*imuRegister = MPU6050_READ_WRITE_REGISTER;
    for (uint8_t index; index < size; ++index)
    {
        writeSuccess = writeByteInRegister(imuRegister, data[index]);
        if (!writeSuccess)
        {
            ROS_ERROR("Failed to write data!");
            break;
        }
    }*/

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

bool Imu::writeWordInRegister(uint8_t registerToWrite, uint16_t value)
{
    hal_pigpio::hal_pigpioI2cWriteWordData i2cWriteWordDataSrv;

    i2cWriteWordDataSrv.request.handle = imuHandle;
    i2cWriteWordDataSrv.request.deviceRegister = registerToWrite;
    i2cWriteWordDataSrv.request.value = value;

    imuClients->getWriteWordDataClientHandle()->call(i2cWriteWordDataSrv);

    if (i2cWriteWordDataSrv.response.hasSucceeded)
    {
        return true;
    }
    else
    {
        ROS_ERROR("Failed to write word!");
        return false;
    }
}

bool Imu::writeDataBlock(uint8_t registerToWrite, std::vector<uint8_t> data)
{
    hal_pigpio::hal_pigpioI2cWriteBlockData i2cWriteBlockDataSrv;

    i2cWriteBlockDataSrv.request.handle = imuHandle;
    i2cWriteBlockDataSrv.request.deviceRegister = registerToWrite;
    i2cWriteBlockDataSrv.request.length = data.size();

    for (uint8_t index = 0; index < data.size(); index++)
    {
        i2cWriteBlockDataSrv.request.dataBlock.push_back(data.at(index));
    }

    imuClients->getWriteBlockDataClientHandle()->call(i2cWriteBlockDataSrv);

    if (i2cWriteBlockDataSrv.response.hasSucceeded)
    {
        return true;
    }
    else
    {
        ROS_ERROR("Failed to write data block!");
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

int32_t Imu::readWordFromRegister(uint8_t registerToRead)
{
    hal_pigpio::hal_pigpioI2cReadWordData i2cReadWordDataSrv;

    i2cReadWordDataSrv.request.handle = imuHandle;
    i2cReadWordDataSrv.request.deviceRegister = registerToRead;

    imuClients->getReadWordDataClientHandle()->call(i2cReadWordDataSrv);

    if (i2cReadWordDataSrv.response.hasSucceeded)
    {
        return i2cReadWordDataSrv.response.value;
    }
    else
    {
        ROS_ERROR("Failed to read word!");
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
    valueRead = readWordFromRegister(MPU6050_FIFO_COUNT_H_REGISTER);
    if (valueRead >= 0)
    {
        fifoCount = valueRead;
        ROS_INFO("Successfully read the number of bytes in the FIFO.");
    }
    else
    {
        ROS_ERROR("Failed to read the number of bytes in the FIFO!");
    }
    /*valueRead = readByteFromRegister(MPU6050_FIFO_COUNT_H_REGISTER);
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
    }*/

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

    ros::Rate loop_rate(0.5);

    while (ros::ok())
    {
        imu.readMpuData();
        imu.publishMessage();

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}