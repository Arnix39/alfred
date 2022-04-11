#ifndef HAL_PIGPIO_IMU
#define HAL_PIGPIO_IMU

#include "ros/ros.h"
#include <vector>

#define _USE_MATH_DEFINES
#include <cmath>

// Pigpio library
#include <pigpiod_if2.h>

// Services and messages headers (generated)
#include "hal_pigpio/hal_pigpioI2cImuReading.h"
#include "hal_pigpio/hal_pigpioAnglesMsg.h"

#define I2C_BUFFER_MAX_BYTES 32

#define MPU6050_POWER_MANAGEMENT_1_REGISTER 0x6B

#define MPU6050_USER_CONTROL_REGISTER 0x6A
#define MPU6050_FIFO_RESET_BIT 2
#define MPU6050_INTERRUPT_STATUS_REGISTER 0x3A
#define MPU6050_FIFO_OVERFLOW 0x10
#define MPU6050_FIFO_COUNT_H_REGISTER 0x72
#define MPU6050_FIFO_COUNT_L_REGISTER 0x73
#define MPU6050_FIFO_REGISTER 0x74
#define MPU6050_MAX_FIFO_SAMPLES 1024
#define MPU6050_MAX_QUATERNIONS_SAMPLES 160
#define MPU6050_DMP_FIFO_QUAT_SIZE 16

class PigpioImu
{
private:
    int pigpioHandle;
    int32_t i2cHandle;
    std::vector<uint32_t> quaternions;
    bool isImuReady;
    ros::ServiceServer imuReadingService;
    ros::Timer readQuaternionsAndPublishAnglesTimer;
    ros::Publisher anglesPublisher;

public:
    PigpioImu(ros::NodeHandle *node, int pigpioHandle);
    ~PigpioImu() = default;
    void readImuData(void);
    void publishAngles(void);
    void readQuaternionsAndPublishAngles(const ros::TimerEvent &event);
    void resetFifo(void);
    uint16_t readFifoCount(void);
    bool isFifoOverflowed(void);
    bool i2cImuReading(hal_pigpio::hal_pigpioI2cImuReading::Request &req,
                       hal_pigpio::hal_pigpioI2cImuReading::Response &res);
};

#endif