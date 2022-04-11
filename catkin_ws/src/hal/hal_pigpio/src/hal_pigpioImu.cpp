#include "hal_pigpioImu.hpp"

PigpioImu::PigpioImu(ros::NodeHandle *node, int pigpioHandle) : pigpioHandle(pigpioHandle),
                                                                i2cHandle(-1),
                                                                quaternions({0, 0, 0, 0}),
                                                                readQuaternionsAndPublishAnglesTimer(node->createTimer(ros::Duration(0.005), &PigpioImu::readQuaternionsAndPublishAngles, this)),
                                                                isImuReady(false),
                                                                imuReadingService(node->advertiseService("hal_pigpioI2cImuReading", &PigpioImu::i2cImuReading, this)),
                                                                anglesPublisher(node->advertise<hal_pigpio::hal_pigpioAnglesMsg>("hal_pigpioAngles", 1000))
{
}

void PigpioImu::resetFifo()
{
    int16_t valueRead;

    valueRead = i2c_read_byte_data(pigpioHandle, i2cHandle, MPU6050_USER_CONTROL_REGISTER);
    if (valueRead < 0)
    {
        ROS_ERROR("Failed to reset FIFO!");
    }
    else if (i2c_write_byte_data(pigpioHandle, i2cHandle, MPU6050_USER_CONTROL_REGISTER, (static_cast<uint8_t>(valueRead) | (1 << MPU6050_FIFO_RESET_BIT))) != 0)
    {
        ROS_ERROR("Failed to reset FIFO!");
    }
}

uint16_t PigpioImu::readFifoCount()
{
    int16_t valueRead;
    uint16_t fifoCount;

    valueRead = i2c_read_byte_data(pigpioHandle, i2cHandle, MPU6050_FIFO_COUNT_H_REGISTER);
    if (valueRead < 0)
    {
        ROS_ERROR("Failed to read the number of bytes in the FIFO!");
        return 0;
    }
    else
    {
        fifoCount = static_cast<uint16_t>(valueRead) << 8;
    }

    valueRead = i2c_read_byte_data(pigpioHandle, i2cHandle, MPU6050_FIFO_COUNT_L_REGISTER);
    if (valueRead < 0)
    {
        ROS_ERROR("Failed to read the number of bytes in the FIFO!");
        return 0;
    }
    else
    {
        fifoCount += static_cast<uint16_t>(valueRead);
    }

    return fifoCount;
}

bool PigpioImu::isFifoOverflowed(void)
{
    int16_t interruptStatus;

    interruptStatus = i2c_read_byte_data(pigpioHandle, i2cHandle, MPU6050_INTERRUPT_STATUS_REGISTER);
    if (interruptStatus < 0)
    {
        ROS_ERROR("Failed to read interrupt status!");
    }
    else if (static_cast<uint8_t>(interruptStatus) & MPU6050_FIFO_OVERFLOW)
    {
        return true;
    }

    return false;
}

void PigpioImu::readImuData(void)
{
    uint16_t fifoCount = 0;
    char fifoData[I2C_BUFFER_MAX_BYTES];

    if (isFifoOverflowed())
    {
        ROS_ERROR("FIFO has overflowed!");
        resetFifo();
    }
    else
    {
        fifoCount = readFifoCount();

        if (fifoCount >= MPU6050_DMP_FIFO_QUAT_SIZE)
        {
            if (i2c_read_i2c_block_data(pigpioHandle, i2cHandle, MPU6050_FIFO_REGISTER, fifoData, MPU6050_DMP_FIFO_QUAT_SIZE) == MPU6050_DMP_FIFO_QUAT_SIZE)
            {
                quaternions.push_back((static_cast<int32_t>(fifoData[0]) << 24) | (static_cast<int32_t>(fifoData[1]) << 16) | (static_cast<int32_t>(fifoData[2]) << 8) | fifoData[3]);
                quaternions.push_back((static_cast<int32_t>(fifoData[4]) << 24) | (static_cast<int32_t>(fifoData[5]) << 16) | (static_cast<int32_t>(fifoData[6]) << 8) | fifoData[7]);
                quaternions.push_back((static_cast<int32_t>(fifoData[8]) << 24) | (static_cast<int32_t>(fifoData[9]) << 16) | (static_cast<int32_t>(fifoData[10]) << 8) | fifoData[11]);
                quaternions.push_back((static_cast<int32_t>(fifoData[12]) << 24) | (static_cast<int32_t>(fifoData[13]) << 16) | (static_cast<int32_t>(fifoData[14]) << 8) | fifoData[15]);
            }
            else
            {
                ROS_ERROR("Failed to read FIFO!");
                resetFifo();
                return;
            }

            if (fifoCount > MPU6050_MAX_QUATERNIONS_SAMPLES)
            {
                resetFifo();
            }
        }
        else
        {
            ROS_INFO("Not enough samples in FIFO.");
        }
    }
}

void PigpioImu::publishAngles(void)
{
    /*hal_pigpio::hal_pigpioAnglesMsg message;

    message.angles = quaternions;
    anglesPublisher.publish(message);*/

    float phi = 0.0f;
    float theta = 0.0f;
    float psi = 0.0f;

    std::vector<float> quaternionsFloat;
    std::vector<float> gravity;

    if (quaternions.size() >= 4)
    {
        quaternionsFloat.push_back(static_cast<float>(quaternions.at(0)) / 1073741824.0f);
        quaternionsFloat.push_back(static_cast<float>(quaternions.at(1)) / 1073741824.0f);
        quaternionsFloat.push_back(static_cast<float>(quaternions.at(2)) / 1073741824.0f);
        quaternionsFloat.push_back(static_cast<float>(quaternions.at(3)) / 1073741824.0f);

        // phi (x-axis rotation)
        float tanPhi = 2 * (quaternionsFloat.at(2) * quaternionsFloat.at(3) - quaternionsFloat.at(0) * quaternionsFloat.at(1));
        float quadrantPhi = 2 * (quaternionsFloat.at(0) * quaternionsFloat.at(0) + quaternionsFloat.at(3) * quaternionsFloat.at(3)) - 1;
        phi = std::atan2(tanPhi, quadrantPhi) * 180 / M_PI;

        // theta (y-axis rotation)
        float sinTheta = 2 * (quaternionsFloat.at(1) * quaternionsFloat.at(3) + quaternionsFloat.at(0) * quaternionsFloat.at(2));
        theta = -std::asin(sinTheta) * 180 / M_PI;

        // psi (z-axis rotation)
        float tanPsi = 2 * (quaternionsFloat.at(1) * quaternionsFloat.at(2) - quaternionsFloat.at(0) * quaternionsFloat.at(3));
        float quadrantPsi = 2 * (quaternionsFloat.at(0) * quaternionsFloat.at(0) + quaternionsFloat.at(1) * quaternionsFloat.at(1)) - 1;
        psi = std::atan2(tanPsi, quadrantPsi) * 180 / M_PI;

        ROS_INFO("phi: %lf, theta: %lf, psi: %lf.", phi, theta, psi);

        quaternions.clear();
    }
}

void PigpioImu::readQuaternionsAndPublishAngles(const ros::TimerEvent &event)
{
    if (isImuReady)
    {
        readImuData();
        publishAngles();
    }
}

bool PigpioImu::i2cImuReading(hal_pigpio::hal_pigpioI2cImuReading::Request &req,
                              hal_pigpio::hal_pigpioI2cImuReading::Response &res)
{
    isImuReady = req.isImuReady;
    i2cHandle = req.imuHandle;
    
    if (isImuReady)
    {
        resetFifo();
    }

    return true;
}