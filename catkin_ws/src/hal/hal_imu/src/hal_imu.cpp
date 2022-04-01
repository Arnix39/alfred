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

/* Subscriber interface implementation */
ImuSubscribersRos::ImuSubscribersRos(ros::NodeHandle *node) : nodeHandle(node)
{
}

void ImuSubscribersRos::subscribe(Imu *imu)
{
    imuImuI2cInitHBSubRos = nodeHandle->subscribe("hal_imuI2cHeartbeatMsg", 1000, &Imu::imuI2cInitHeartbeatCallback, imu);
}

/* Services clients interface implementation */
ImuClientsRos::ImuClientsRos(ros::NodeHandle *node) : i2cReadByteDataClientRos(node->serviceClient<hal_pigpio::hal_pigpioI2cReadByteData>("hal_pigpioI2cReadByteData")),
                                                      i2cWriteByteDataClientRos(node->serviceClient<hal_pigpio::hal_pigpioI2cWriteByteData>("hal_pigpioI2cWriteByteData")),
                                                      i2cReadWordDataClientRos(node->serviceClient<hal_pigpio::hal_pigpioI2cReadWordData>("hal_pigpioI2cReadWordData")),
                                                      i2cWriteWordDataClientRos(node->serviceClient<hal_pigpio::hal_pigpioI2cWriteWordData>("hal_pigpioI2cWriteWordData")),
                                                      i2cReadBlockDataClientRos(node->serviceClient<hal_pigpio::hal_pigpioI2cReadBlockData>("hal_pigpioI2cReadBlockData")),
                                                      i2cWriteBlockDataClientRos(node->serviceClient<hal_pigpio::hal_pigpioI2cWriteBlockData>("hal_pigpioI2cWriteBlockData")),
                                                      i2cImuReadingClientRos(node->serviceClient<hal_pigpio::hal_pigpioI2cImuReading>("hal_pigpioI2cImuReading")),
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

ros::ServiceClient *ImuClientsRos::getReadBlockDataClientHandle()
{
    return &i2cReadBlockDataClientRos;
}

ros::ServiceClient *ImuClientsRos::getWriteBlockDataClientHandle()
{
    return &i2cWriteBlockDataClientRos;
}

ros::ServiceClient *ImuClientsRos::getGetHandleClientHandle()
{
    return &i2cGetHandleClientRos;
}

ros::ServiceClient *ImuClientsRos::getImuReadingClientHandle()
{
    return &i2cImuReadingClientRos;
}

/* IMU implementation */
Imu::Imu(ImuPublisher *imuMessagePublisher, ImuClients *imuServiceClients, ImuSubscribers *imuSubscribers) : imuPublisher(imuMessagePublisher),
                                                                                                             imuClients(imuServiceClients),
                                                                                                             angle(0),
                                                                                                             dmpEnabled(false),
                                                                                                             imuSubs(imuSubscribers),
                                                                                                             i2cInitialised(false),
                                                                                                             isStarted(false)
{
    imuSubs->subscribe(this);
}

void Imu::getI2cHandle(void)
{
    hal_imu::hal_imuGetHandle i2cGetHandleSrv;
    imuClients->getGetHandleClientHandle()->call(i2cGetHandleSrv);
    imuHandle = i2cGetHandleSrv.response.handle;
}

void Imu::imuI2cInitHeartbeatCallback(const hal_imu::hal_imuI2cHeartbeatMsg &msg)
{
    i2cInitialised = msg.isAlive;
}

bool Imu::isI2cInitialised(void)
{
    return i2cInitialised;
}

void Imu::init(void)
{
    resetImu();
    setClockSource();
    setMpuRate(MPU6050_DMP_SAMPLE_RATE);
    setConfiguration();
    writeDmp();
    setDmpRate(MPU6050_DMP_SAMPLE_RATE);
    writeOrientationMatrix();
    configureDmpFeatures();
    enableDmp();
    resetFifo();
    //  calibrateAccelerometer();
}

void Imu::resetImu(void)
{
    ROS_INFO("IMU resetting...");

    /* Reset MPU6050 */
    if (!writeByteInRegister(MPU6050_POWER_MANAGEMENT_1_REGISTER, MPU6050_RESET))
    {
        ROS_ERROR("Failed to reset IMU because chip couldn't be resetted.");
        return;
    }

    ros::Duration(0.1).sleep();

    /* Reset signal paths */
    if (!writeByteInRegister(MPU6050_SIGNAL_PATH_RESET_REGISTER, MPU6050_SIGNAL_PATH_RESET))
    {
        ROS_ERROR("Failed to reset IMU because signal paths couldn't be resetted.");
        return;
    }

    ros::Duration(0.1).sleep();

    /* Disable sleep mode */
    if (!writeByteInRegister(MPU6050_POWER_MANAGEMENT_1_REGISTER, 0x00))
    {
        ROS_ERROR("Failed to reset IMU because sleep mode couldn't be disabled.");
        return;
    }

    ROS_INFO("Successfully resetted IMU.");
}

void Imu::setClockSource(void)
{
    bool writeSuccess;

    writeSuccess = writeByteInRegister(MPU6050_POWER_MANAGEMENT_1_REGISTER, MPU6050_CLOCK_SOURCE_PLL_X);

    if (writeSuccess)
    {
        ROS_INFO("Successfully enabled PLL_X clock source.");
    }
    else
    {
        ROS_ERROR("Failed to enable PLL_X clock source.");
    }
}

void Imu::setConfiguration(void)
{
    bool writeSuccess;

    writeSuccess = writeByteInRegister(MPU6050_CONFIGURATION_REGISTER, MPU6050_DLPF_BANDWITH_42);

    if (writeSuccess)
    {
        ROS_INFO("Successfully wrote configuration of MPU.");
    }
    else
    {
        ROS_ERROR("Failed to write configuration of MPU.");
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

    imuDmpWritingClient.waitForResult();
    if (imuDmpWritingClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("DMP code written successfully.");
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

    if (!writeDataToDmp(MPU6050_DMP_ORIENTATION_MATRIX_GYRO_AXES_BANK, MPU6050_DMP_ORIENTATION_MATRIX_GYRO_AXES_ADDRESS, dmpOrientationMatrixGyroAxes))
    {
        writeSuccess = false;
        ROS_ERROR("Failed to write the gyroscope axes of the orientation matrix!");
    }

    if (!writeDataToDmp(MPU6050_DMP_ORIENTATION_MATRIX_GYRO_SIGNS_BANK, MPU6050_DMP_ORIENTATION_MATRIX_GYRO_SIGNS_ADDRESS, dmpOrientationMatrixGyroSigns))
    {
        writeSuccess = false;
        ROS_ERROR("Failed to write the gyroscope signs of the orientation matrix!");
    }

    if (!writeDataToDmp(MPU6050_DMP_ORIENTATION_MATRIX_ACCEL_AXES_BANK, MPU6050_DMP_ORIENTATION_MATRIX_ACCEL_AXES_ADDRESS, dmpOrientationMatrixAccelAxes))
    {
        writeSuccess = false;
        ROS_ERROR("Failed to write the accelerometer axes of the orientation matrix!");
    }

    if (!writeDataToDmp(MPU6050_DMP_ORIENTATION_MATRIX_ACCEL_SIGNS_BANK, MPU6050_DMP_ORIENTATION_MATRIX_ACCEL_SIGNS_ADDRESS, dmpOrientationMatrixAccelSigns))
    {
        writeSuccess = false;
        ROS_ERROR("Failed to write the accelerometer signs of the orientation matrix!");
    }

    if (writeSuccess)
    {
        ROS_INFO("Successfully wrote orientation matrix.");
    }
}

void Imu::resetFifo()
{
    if (!writeBitInRegister(MPU6050_USER_CONTROL_REGISTER, MPU6050_FIFO_RESET_BIT, 1))
    {
        ROS_ERROR("Failed to reset FIFO!");
    }
    else
    {
        ROS_INFO("Successfully resetted FIFO.");
    }
}

void Imu::enableDmp(void)
{
    /* Enable FIFO */
    if (!writeByteInRegister(MPU6050_USER_CONTROL_REGISTER, MPU6050_FIFO_ENABLE))
    {
        ROS_ERROR("Failed to enable DMP because FIFO could not be enabled!");
        return;
    }

    /* Enable DMP */
    if (!writeByteInRegister(MPU6050_USER_CONTROL_REGISTER, MPU6050_DMP_EMABLE))
    {
        ROS_ERROR("Failed to enable DMP!");
        return;
    }

    dmpEnabled = true;
    ROS_INFO("Successfully enabled DMP.");
}

void Imu::calibrateAccelerometer(void)
{
}

void Imu::configureDmpFeatures(void)
{
    std::vector<uint8_t> dmpSendSensorData(MPU6050_DMP_FEATURE_SEND_SENSOR_DATA_SIZE, 0xA3);
    std::vector<uint8_t> dmpSendGestureData{0xD8};
    std::vector<uint8_t> gyroscopeCalibrationEnable{0xb8, 0xaa, 0xb3, 0x8d, 0xb4, 0x98, 0x0d, 0x35, 0x5d};
    std::vector<uint8_t> dmpSendTapData{0xD8};
    std::vector<uint8_t> dmpSendAndroidOrientation{0xD8};
    std::vector<uint8_t> dmpQuaternionDisable(MPU6050_DMP_FEATURE_QUATERNION_SIZE, 0x8B);
    std::vector<uint8_t> dmp6AxisQuaternionEnable{0x20, 0x28, 0x30, 0x38};

    bool configurationSuccessful = true;

    if (!writeDataToDmp(MPU6050_DMP_FEATURE_SEND_SENSOR_DATA_BANK, MPU6050_DMP_FEATURE_SEND_SENSOR_DATA_ADDRESS, dmpSendSensorData))
    {
        configurationSuccessful = false;
        ROS_ERROR("Error while disabling sensor data on DMP!");
    }

    if (!writeDataToDmp(MPU6050_DMP_FEATURE_SEND_GESTURE_DATA_BANK, MPU6050_DMP_FEATURE_SEND_GESTURE_DATA_ADDRESS, dmpSendGestureData))
    {
        configurationSuccessful = false;
        ROS_ERROR("Error while disabling gesture data on DMP!");
    }

    if (!writeDataToDmp(MPU6050_CFG_MOTION_BIAS_BANK, MPU6050_CFG_MOTION_BIAS_ADDRESS, gyroscopeCalibrationEnable))
    {
        configurationSuccessful = false;
        ROS_ERROR("Error while enabling gyroscope calibration on DMP!");
    }

    if (!writeDataToDmp(MPU6050_DMP_FEATURE_SEND_TAP_DATA_BANK, MPU6050_DMP_FEATURE_SEND_TAP_DATA_ADDRESS, dmpSendTapData))
    {
        configurationSuccessful = false;
        ROS_ERROR("Error while disabling tap data on DMP!");
    }

    if (!writeDataToDmp(MPU6050_DMP_FEATURE_SEND_ANDROID_ORIENTATION_BANK, MPU6050_DMP_FEATURE_SEND_ANDROID_ORIENTATION_ADDRESS, dmpSendAndroidOrientation))
    {
        configurationSuccessful = false;
        ROS_ERROR("Error while disabling android orientation on DMP!");
    }

    if (!writeDataToDmp(MPU6050_DMP_FEATURE_QUATERNION_BANK, MPU6050_DMP_FEATURE_QUATERNION_ADDRESS, dmpQuaternionDisable))
    {
        configurationSuccessful = false;
        ROS_ERROR("Error while disabling quaternion computation on DMP!");
    }

    resetFifo();

    if (!writeDataToDmp(MPU6050_DMP_FEATURE_6X_LP_QUAT_BANK, MPU6050_DMP_FEATURE_6X_LP_QUAT_ADDRESS, dmp6AxisQuaternionEnable))
    {
        configurationSuccessful = false;
        ROS_ERROR("Error while enabling 6 axis quaternion computation on DMP!");
    }

    resetFifo();

    if (configurationSuccessful)
    {
        ROS_INFO("Successfully configured DMP features.");
    }
    else
    {
        ROS_ERROR("Failed to configure DMP features.");
    }
}

void Imu::setDmpRate(uint16_t rate)
{
    uint16_t div;
    std::vector<uint8_t> div_vec;

    if (rate > MPU6050_DMP_SAMPLE_RATE)
    {
        div = MPU6050_DMP_SAMPLE_RATE / MPU6050_DMP_SAMPLE_RATE - 1;
    }
    else
    {
        div = MPU6050_DMP_SAMPLE_RATE / rate - 1;
    }

    div_vec.push_back(static_cast<uint8_t>(div >> 8));
    div_vec.push_back(static_cast<uint8_t>(div & 0xFF));

    writeDataToDmp(MPU6050_DMP_SAMPLE_RATE_BANK, MPU6050_DMP_SAMPLE_RATE_ADDRESS, div_vec);
}

void Imu::setMpuRate(uint16_t rate)
{
    uint8_t div;

    if (rate > MPU6050_MAX_SAMPLE_RATE)
    {
        rate = MPU6050_MAX_SAMPLE_RATE;
    }
    else if (rate < MPU6050_MIN_SAMPLE_RATE)
    {
        rate = MPU6050_MIN_SAMPLE_RATE;
    }

    div = MPU6050_MAX_SAMPLE_RATE / rate - 1;

    writeByteInRegister(MPU6050_SAMPLE_RATE_REGISTER, div);
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

bool Imu::writeDataToDmp(uint8_t bank, uint8_t addressInBank, std::vector<uint8_t> data)
{
    if (!writeByteInRegister(MPU6050_BANK_SELECTION_REGISTER, bank))
    {
        ROS_ERROR("Failed to write bank!");
        return false;
    }

    if (!writeByteInRegister(MPU6050_ADDRESS_IN_BANK_REGISTER, addressInBank))
    {
        ROS_ERROR("Failed to write address!");
        return false;
    }

    if (!writeDataBlock(MPU6050_READ_WRITE_REGISTER, data))
    {
        ROS_ERROR("Failed to write data!");
        return false;
    }

    return true;
}

void Imu::startImuReading()
{
    hal_pigpio::hal_pigpioI2cImuReading i2cImuReadingSrv;

    i2cImuReadingSrv.request.isImuReady = true;
    i2cImuReadingSrv.request.imuHandle = imuHandle;

    imuClients->getImuReadingClientHandle()->call(i2cImuReadingSrv);
}

void Imu::stopImuReading()
{
    hal_pigpio::hal_pigpioI2cImuReading i2cImuReadingSrv;

    i2cImuReadingSrv.request.isImuReady = false;
    i2cImuReadingSrv.request.imuHandle = imuHandle;

    imuClients->getImuReadingClientHandle()->call(i2cImuReadingSrv);
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

std::vector<uint8_t> Imu::readBlockFromRegister(uint8_t registerToRead, uint8_t bytesToRead)
{
    hal_pigpio::hal_pigpioI2cReadBlockData i2cReadBlockDataSrv;

    i2cReadBlockDataSrv.request.handle = imuHandle;
    i2cReadBlockDataSrv.request.deviceRegister = registerToRead;
    i2cReadBlockDataSrv.request.length = bytesToRead;

    imuClients->getReadBlockDataClientHandle()->call(i2cReadBlockDataSrv);

    if (i2cReadBlockDataSrv.response.hasSucceeded)
    {
        return i2cReadBlockDataSrv.response.dataBlock;
    }
    else
    {
        ROS_ERROR("Failed to read data block!");
        return {};
    }
}

void Imu::publishMessage(void)
{
    hal_imu::hal_imuMsg message;

    message.angleInDeg = angle;
    imuPublisher->publish(message);
}

void Imu::starts(void)
{
    isStarted = true;
}

bool Imu::isNotStarted(void)
{
    return !isStarted;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hal_imu");
    ros::NodeHandle node;

    ImuPublisherRos imuMessagePublisherRos(&node);
    ImuClientsRos imuServiceClientsRos(&node);
    ImuSubscribersRos imuSubscribersRos(&node);

    Imu imu(&imuMessagePublisherRos, &imuServiceClientsRos, &imuSubscribersRos);

    ROS_INFO("imu node waiting for I2C communication to be ready...");
    while (ros::ok())
    {
        if (imu.isNotStarted() && imu.isI2cInitialised())
        {
            imu.getI2cHandle();
            ROS_INFO("imu I2C communication ready.");
            imu.init();
            imu.starts();
            ROS_INFO("imu node initialised.");
            imu.startImuReading();
        }

        ros::spinOnce();
    }

    return 0;
}