#include "hal_pigpioImu.hpp"

PigpioImu::PigpioImu(ros::NodeHandle *node, int pigpioHandle) : pigpioHandle(pigpioHandle),
                                                                i2cHandle(-1),
                                                                quaternions({}),
                                                                readAndPublishQuaternionsTimer(node->createTimer(ros::Duration(0.01), &PigpioImu::readAndPublishQuaternions, this)),
                                                                isImuReady(false),
                                                                imuReadingService(node->advertiseService("hal_pigpioI2cImuReading", &PigpioImu::i2cImuReading, this))
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
    else if (i2c_write_byte_data(pigpioHandle, i2cHandle, MPU6050_USER_CONTROL_REGISTER, ((uint8_t)valueRead | (1 << MPU6050_FIFO_RESET_BIT))) == 0)
    {
        ROS_INFO("Successfully resetted FIFO.");
    }
    else
    {
        ROS_ERROR("Failed to reset FIFO!");
    }
}

void PigpioImu::readImuData(void)
{
    int16_t interruptStatus;
    int16_t valueRead;
    uint16_t fifoCount = 0;
    std::vector<uint8_t> fifoData;
    char buffer[I2C_BUFFER_MAX_BYTES];
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

        ROS_INFO("Number of samples in FIFO: %u.", fifoCount);

        if (fifoCount >= MPU6050_DMP_FIFO_QUAT_SIZE)
        {
            result = i2c_read_i2c_block_data(pigpioHandle, i2cHandle, MPU6050_FIFO_REGISTER, buffer, MPU6050_DMP_FIFO_QUAT_SIZE);
            resetFifo();

            if ((result > 0) && (result == MPU6050_DMP_FIFO_QUAT_SIZE))
            {
                for (uint8_t index = 0; index < result; index++)
                {
                    fifoData.push_back(buffer[index]);
                }
            }
            else
            {
                ROS_ERROR("Failed to read FIFO!");
                return;
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
}

void PigpioImu::readAndPublishQuaternions(const ros::TimerEvent &event)
{
    if (isImuReady)
    {
        readImuData();
    }
    publishQuaternions();
}

bool PigpioImu::i2cImuReading(hal_pigpio::hal_pigpioI2cImuReading::Request &req,
                              hal_pigpio::hal_pigpioI2cImuReading::Response &res)
{
    isImuReady = req.isImuReady;
    i2cHandle = req.imuHandle;

    return true;
}