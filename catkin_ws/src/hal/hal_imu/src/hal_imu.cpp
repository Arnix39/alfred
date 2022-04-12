#include "hal_imu.hpp"
#include "hal_imuInterfaces.hpp"
#include "hal_imuMPU6050.hpp"

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
Imu::Imu(ImuClients *imuServiceClients, ImuSubscribers *imuSubscribers) : imuClients(imuServiceClients),
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
    setAccelerometerSensitivity();
    setGyroscopeSensitivity();
    setConfiguration();
    setMpuRate(MPU6050_DMP_SAMPLE_RATE);
    writeDmp();
    setDmpRate(MPU6050_DMP_SAMPLE_RATE);
    setAccelerometerOffsets();
    setGyroscopeOffsets();
    configureDmpFeatures();
    enableDmp();
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
    if (!writeByteInRegister(MPU6050_POWER_MANAGEMENT_1_REGISTER, MPU6050_CLOCK_SOURCE_PLL_X))
    {
        ROS_ERROR("Failed to enable PLL_X clock source.");
    }
    else
    {
        ROS_INFO("Successfully enabled PLL_X clock source.");
    }
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

    if (!writeByteInRegister(MPU6050_SAMPLE_RATE_REGISTER, div))
    {
        ROS_ERROR("Failed to set MPU sample rate.");
    }
    else
    {
        ROS_INFO("Successfully set MPU sample rate.");
    }
}

void Imu::setConfiguration(void)
{
    if (!writeByteInRegister(MPU6050_CONFIGURATION_REGISTER, MPU6050_DLPF_BANDWITH_188))
    {
        ROS_ERROR("Failed to write configuration of MPU.");
    }
    else
    {
        ROS_INFO("Successfully wrote configuration of MPU.");
    }
}

void Imu::setAccelerometerSensitivity(void)
{
    if (!writeByteInRegister(MPU6050_ACCELEROMETER_CONFIGURATION_REGISTER, MPU6050_ACCELEROMETER_FULL_SENSITIVITY))
    {
        ROS_ERROR("Failed to set accelerometer sensitivity!");
    }
    else
    {
        ROS_INFO("Successfully set accelerometer sensitivity.");
    }
}

void Imu::setGyroscopeSensitivity(void)
{
    if (!writeByteInRegister(MPU6050_GYROSCOPE_CONFIGURATION_REGISTER, MPU6050_GYROSCOPE_FULL_SENSITIVITY))
    {
        ROS_ERROR("Failed to set gyroscope sensitivity!");
    }
    else
    {
        ROS_INFO("Successfully set gyroscope sensitivity.");
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

void Imu::setDmpRate(uint16_t rate)
{
    uint16_t div;
    std::vector<uint8_t> div_vec;
    std::vector<uint8_t> dmpRegisterDiviserData{0xFE, 0xF2, 0xAB, 0xC4, 0xAA, 0xF1, 0xDF, 0xDF, 0xBB, 0xAF, 0xDF, 0xDF};

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

    if (!writeDataToDmp(MPU6050_DMP_SAMPLE_RATE_BANK, MPU6050_DMP_SAMPLE_RATE_ADDRESS, div_vec))
    {
        ROS_ERROR("Failed to write DMP sample rate.");
        return;
    }

    if (!writeDataToDmp(MPU6050_DMP_DIVISER_BANK, MPU6050_DMP_DIVISER_ADDRESS, dmpRegisterDiviserData))
    {
        ROS_ERROR("Failed to write DMP diviser data.");
        return;
    }

    ROS_INFO("Successfully set DMP sample rate.");
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

void Imu::setAccelerometerOffsets(void)
{
    SensorBias accelerometerXBias{'X', IMU_ACCELEROMETER_X_OFFSET, MPU6050_ACCELEROMETER_X_OFFSET_MSB_REGISTER, MPU6050_ACCELEROMETER_X_OFFSET_LSB_REGISTER};
    SensorBias accelerometerYBias{'Y', IMU_ACCELEROMETER_Y_OFFSET, MPU6050_ACCELEROMETER_Y_OFFSET_MSB_REGISTER, MPU6050_ACCELEROMETER_Y_OFFSET_LSB_REGISTER};
    SensorBias accelerometerZBias{'Z', IMU_ACCELEROMETER_Z_OFFSET, MPU6050_ACCELEROMETER_Z_OFFSET_MSB_REGISTER, MPU6050_ACCELEROMETER_Z_OFFSET_LSB_REGISTER};
    
    const std::vector<SensorBias> accelerometerBiases{accelerometerXBias, accelerometerYBias, accelerometerZBias};
    
    if (!writeSensorBiases(accelerometerBiases))
    {
        ROS_ERROR("Failed to set accelerometer offsets.");
    }
    else
    {
        ROS_INFO("Successfully set accelerometer offsets.");
    }
}

void Imu::setGyroscopeOffsets(void)
{
    SensorBias gyroscopeXBias{'X', IMU_GYROSCOPE_X_OFFSET, MPU6050_GYROSCOPE_X_OFFSET_MSB_REGISTER, MPU6050_GYROSCOPE_X_OFFSET_LSB_REGISTER};
    SensorBias gyroscopeYBias{'Y', IMU_GYROSCOPE_Y_OFFSET, MPU6050_GYROSCOPE_Y_OFFSET_MSB_REGISTER, MPU6050_GYROSCOPE_Y_OFFSET_LSB_REGISTER};
    SensorBias gyroscopeZBias{'Z', IMU_GYROSCOPE_Z_OFFSET, MPU6050_GYROSCOPE_Z_OFFSET_MSB_REGISTER, MPU6050_GYROSCOPE_Z_OFFSET_LSB_REGISTER};
    
    const std::vector<SensorBias> gyroscopeBiases{gyroscopeXBias, gyroscopeYBias, gyroscopeZBias};
    
    if (!writeSensorBiases(gyroscopeBiases))
    {
        ROS_ERROR("Failed to set gyroscope offsets.");
    }
    else
    {
        ROS_INFO("Successfully set gyroscope offsets.");
    }
}

bool Imu::writeSensorBiases(const std::vector<SensorBias> sensorBiases)
{
    for (auto sensorBias : sensorBiases)
    {
        uint8_t sensorBiasMsb = static_cast<uint8_t>((sensorBias.bias >> 8) & 0xFF);
        uint8_t sensorBiasLsb = static_cast<uint8_t>(sensorBias.bias & 0xFF);

        if (!writeByteInRegister(sensorBias.msbRegister, sensorBiasMsb))
        {
            ROS_ERROR("Failed to set sensor %c offset!", sensorBias.axis);
            return false;
        }

        if (!writeByteInRegister(sensorBias.lsbRegister, sensorBiasLsb))
        {
            ROS_ERROR("Failed to set sensor %c offset!", sensorBias.axis);
            return false;
        }
    }

    return true;
}

void Imu::configureDmpFeatures(void)
{
    std::vector<uint8_t> dmpNoSensorData(MPU6050_DMP_FEATURE_SEND_SENSOR_DATA_SIZE, 0xA3);
    std::vector<uint8_t> dmpNoGestureData{0xD8};
    std::vector<uint8_t> gyroscopeCalibrationDisabled{0xB8, 0xAA, 0xAA, 0xAA, 0xB0, 0x88, 0xC3, 0xC5, 0xC7};
    std::vector<uint8_t> dmpNoTapData{0xD8};
    std::vector<uint8_t> dmpNoAndroidOrientation{0xD8};
    std::vector<uint8_t> dmpQuaternionDisabled(MPU6050_DMP_FEATURE_QUATERNION_SIZE, 0x8B);
    std::vector<uint8_t> dmp6AxisQuaternionEnabled{0x20, 0x28, 0x30, 0x38};

    if (!writeDataToDmp(MPU6050_DMP_FEATURE_SEND_SENSOR_DATA_BANK, MPU6050_DMP_FEATURE_SEND_SENSOR_DATA_ADDRESS, dmpNoSensorData))
    {
        ROS_ERROR("Failed to configure DMP features (sensor data disabled)!");
        return;
    }

    if (!writeDataToDmp(MPU6050_DMP_FEATURE_SEND_GESTURE_DATA_BANK, MPU6050_DMP_FEATURE_SEND_GESTURE_DATA_ADDRESS, dmpNoGestureData))
    {
        ROS_ERROR("Failed to configure DMP features (gesture data disabled)!");
        return;
    }

    if (!writeDataToDmp(MPU6050_CFG_MOTION_BIAS_BANK, MPU6050_CFG_MOTION_BIAS_ADDRESS, gyroscopeCalibrationDisabled))
    {
        ROS_ERROR("Failed to configure DMP features (gyroscope calibration disabled)!");
        return;
    }

    if (!writeDataToDmp(MPU6050_DMP_FEATURE_SEND_TAP_DATA_BANK, MPU6050_DMP_FEATURE_SEND_TAP_DATA_ADDRESS, dmpNoTapData))
    {
        ROS_ERROR("Failed to configure DMP features (tap data disabled)!");
        return;
    }

    if (!writeDataToDmp(MPU6050_DMP_FEATURE_SEND_ANDROID_ORIENTATION_BANK, MPU6050_DMP_FEATURE_SEND_ANDROID_ORIENTATION_ADDRESS, dmpNoAndroidOrientation))
    {
        ROS_ERROR("Failed to configure DMP features (android orientation disbaled)!");
        return;
    }

    if (!writeDataToDmp(MPU6050_DMP_FEATURE_QUATERNION_BANK, MPU6050_DMP_FEATURE_QUATERNION_ADDRESS, dmpQuaternionDisabled))
    {
        ROS_ERROR("Failed to configure DMP features (quaternion computation disabled)!");
        return;
    }

    if (!writeDataToDmp(MPU6050_DMP_FEATURE_6X_LP_QUAT_BANK, MPU6050_DMP_FEATURE_6X_LP_QUAT_ADDRESS, dmp6AxisQuaternionEnabled))
    {
        ROS_ERROR("Failed to configure DMP features (6 axis quaternion computation enabled)!");
        return;
    }

    resetFifo();

    ROS_INFO("Successfully configured DMP features.");
}

void Imu::enableDmp(void)
{
    /* Enable DMP and FIFO */
    if (!writeByteInRegister(MPU6050_USER_CONTROL_REGISTER, MPU6050_DMP_EMABLE | MPU6050_FIFO_ENABLE))
    {
        ROS_ERROR("Failed to enable DMP!");
        return;
    }

    resetFifo();

    ROS_INFO("Successfully enabled DMP.");
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

bool Imu::writeBitInRegister(uint8_t registerToWrite, uint8_t bitToWrite, uint8_t valueOfBit)
{
    uint8_t registerValue;
    int16_t valueRead;
    uint8_t newRegisterValue;

    valueRead = readByteFromRegister(registerToWrite);
    if (valueRead < 0)
    {
        return false;
    }
    else
    {
        registerValue = static_cast<uint8_t>(valueRead);
    }

    if (valueOfBit == 1)
    {
        newRegisterValue = registerValue | (1 << bitToWrite);
    }
    else
    {
        newRegisterValue = registerValue & ~(1 << bitToWrite);
    }

    return writeByteInRegister(registerToWrite, newRegisterValue);
}

bool Imu::writeByteInRegister(uint8_t registerToWrite, uint8_t value)
{
    hal_pigpio::hal_pigpioI2cWriteByteData i2cWriteByteDataSrv;

    i2cWriteByteDataSrv.request.handle = imuHandle;
    i2cWriteByteDataSrv.request.deviceRegister = registerToWrite;
    i2cWriteByteDataSrv.request.value = value;

    imuClients->getWriteByteDataClientHandle()->call(i2cWriteByteDataSrv);

    return i2cWriteByteDataSrv.response.hasSucceeded;
}

bool Imu::writeDataBlock(uint8_t registerToWrite, std::vector<uint8_t> data)
{
    hal_pigpio::hal_pigpioI2cWriteBlockData i2cWriteBlockDataSrv;

    i2cWriteBlockDataSrv.request.handle = imuHandle;
    i2cWriteBlockDataSrv.request.deviceRegister = registerToWrite;
    i2cWriteBlockDataSrv.request.length = data.size();

    for (uint8_t index = 0; index < data.size(); ++index)
    {
        i2cWriteBlockDataSrv.request.dataBlock.push_back(data.at(index));
    }

    imuClients->getWriteBlockDataClientHandle()->call(i2cWriteBlockDataSrv);

    return i2cWriteBlockDataSrv.response.hasSucceeded;
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

    ImuClientsRos imuServiceClientsRos(&node);
    ImuSubscribersRos imuSubscribersRos(&node);

    Imu imu(&imuServiceClientsRos, &imuSubscribersRos);

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