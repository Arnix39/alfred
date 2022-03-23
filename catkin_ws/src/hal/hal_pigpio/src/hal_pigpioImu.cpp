#include "hal_pigpioImu.hpp"

PigpioImu::PigpioImu(ros::NodeHandle *node, int pigpioHandle) : pigpioHandle(pigpioHandle),
                                                                i2cHandle(-1),
                                                                quaternions({0, 0, 0, 0}),
                                                                readAndPublishQuaternionsTimer(node->createTimer(ros::Duration(0.01), &PigpioImu::readAndPublishQuaternions, this)),
                                                                isImuReady(false),
                                                                imuReadingService(node->advertiseService("hal_pigpioI2cImuReading", &PigpioImu::i2cImuReading, this)),
                                                                quaternionsPublisher(node->advertise<hal_pigpio::hal_pigpioQuaternionsMsg>("hal_pigpioQuaternions", 1000))
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
    else if (i2c_write_byte_data(pigpioHandle, i2cHandle, MPU6050_USER_CONTROL_REGISTER, ((uint8_t)valueRead | (1 << MPU6050_FIFO_RESET_BIT))) != 0)
    {
        ROS_ERROR("Failed to reset FIFO!");
    }
}

void PigpioImu::readImuData(void)
{
    int16_t interruptStatus;
    int16_t valueRead;
    uint16_t fifoCount = 0;
    char fifoData[I2C_BUFFER_MAX_BYTES];
    int result;

    interruptStatus = i2c_read_byte_data(pigpioHandle, i2cHandle, MPU6050_INTERRUPT_STATUS_REGISTER);
    if (interruptStatus < 0)
    {
        ROS_ERROR("Failed to read interrupt status!");
    }
    else if ((uint8_t)interruptStatus & MPU6050_FIFO_OVERFLOW)
    {
        ROS_ERROR("FIFO has overflowed!");
        resetFifo();
    }
    else
    {
        valueRead = i2c_read_byte_data(pigpioHandle, i2cHandle, MPU6050_FIFO_COUNT_H_REGISTER);
        if (valueRead < 0)
        {
            ROS_ERROR("Failed to read the number of bytes in the FIFO!");
            return;
        }
        else
        {
            fifoCount = ((uint16_t)valueRead << 8);
        }

        valueRead = i2c_read_byte_data(pigpioHandle, i2cHandle, MPU6050_FIFO_COUNT_L_REGISTER);
        if (valueRead < 0)
        {
            ROS_ERROR("Failed to read the number of bytes in the FIFO!");
            return;
        }
        else
        {
            fifoCount += (uint16_t)valueRead;
        }

        if (fifoCount >= MPU6050_DMP_FIFO_QUAT_SIZE)
        {
            result = i2c_read_i2c_block_data(pigpioHandle, i2cHandle, MPU6050_FIFO_REGISTER, fifoData, MPU6050_DMP_FIFO_QUAT_SIZE);

            if ((result > 0) && (result == MPU6050_DMP_FIFO_QUAT_SIZE))
            {
                quaternions.push_back(((uint32_t)fifoData[0] << 24) | ((uint32_t)fifoData[1] << 16) | ((uint32_t)fifoData[2] << 8) | fifoData[3]);
                quaternions.push_back(((uint32_t)fifoData[4] << 24) | ((uint32_t)fifoData[5] << 16) | ((uint32_t)fifoData[6] << 8) | fifoData[7]);
                quaternions.push_back(((uint32_t)fifoData[8] << 24) | ((uint32_t)fifoData[9] << 16) | ((uint32_t)fifoData[10] << 8) | fifoData[11]);
                quaternions.push_back(((uint32_t)fifoData[12] << 24) | ((uint32_t)fifoData[13] << 16) | ((uint32_t)fifoData[14] << 8) | fifoData[15]);
            }
            else
            {
                ROS_ERROR("Failed to read FIFO!");
                return;
            }

            if (fifoCount > MPU6050_MAX_FIFO_SAMPLES / 2)
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

void PigpioImu::publishQuaternions(void)
{
    hal_pigpio::hal_pigpioQuaternionsMsg message;

    message.quaternions = quaternions;
    quaternionsPublisher.publish(message);

    double pitch;
    double roll;
    double yaw;

    std::vector<double> quaternionsFloat;

    if (quaternions.size() >= 4)
    {
        quaternionsFloat.push_back((double)quaternions.at(0) / 16384.0f);
        quaternionsFloat.push_back((double)quaternions.at(1) / 16384.0f);
        quaternionsFloat.push_back((double)quaternions.at(2) / 16384.0f);
        quaternionsFloat.push_back((double)quaternions.at(3) / 16384.0f);

        // roll (x-axis rotation)
        double sinr_cosp = 2 * (quaternionsFloat.at(0) * quaternionsFloat.at(1) + quaternionsFloat.at(2) * quaternionsFloat.at(3));
        double cosr_cosp = 1 - 2 * (quaternionsFloat.at(1) * quaternionsFloat.at(1) + quaternionsFloat.at(2) * quaternionsFloat.at(2));
        roll = std::atan2(sinr_cosp, cosr_cosp);

        // pitch (y-axis rotation)
        double sinp = 2 * (quaternionsFloat.at(0) * quaternionsFloat.at(2) - quaternionsFloat.at(3) * quaternionsFloat.at(1));
        if (std::abs(sinp) >= 1)
            pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
        else
            pitch = std::asin(sinp);

        // yaw (z-axis rotation)
        double siny_cosp = 2 * (quaternionsFloat.at(0) * quaternionsFloat.at(3) + quaternionsFloat.at(1) * quaternionsFloat.at(2));
        double cosy_cosp = 1 - 2 * (quaternionsFloat.at(2) * quaternionsFloat.at(2) + quaternionsFloat.at(3) * quaternionsFloat.at(3));
        yaw = std::atan2(siny_cosp, cosy_cosp);

        // ROS_INFO("Quaternions: %d, %d, %d, %d.", quaternions.at(0), quaternions.at(1), quaternions.at(2), quaternions.at(3));
        ROS_INFO("Roll: %lf, pitch: %lf, yaw: %lf.", roll, pitch, yaw);

        quaternions.clear();
    }
}

void PigpioImu::readAndPublishQuaternions(const ros::TimerEvent &event)
{
    if (isImuReady)
    {
        readImuData();
        publishQuaternions();
    }
}

bool PigpioImu::i2cImuReading(hal_pigpio::hal_pigpioI2cImuReading::Request &req,
                              hal_pigpio::hal_pigpioI2cImuReading::Response &res)
{
    isImuReady = req.isImuReady;
    i2cHandle = req.imuHandle;

    return true;
}