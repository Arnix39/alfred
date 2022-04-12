#ifndef HAL_PIGPIO_IMU
#define HAL_PIGPIO_IMU

#include "ros/ros.h"
#include <vector>

#define _USE_MATH_DEFINES
#include <cmath>

#include "hal_mpu6050.hpp"

// Pigpio library
#include <pigpiod_if2.h>

// Services and messages headers (generated)
#include "hal_pigpio/hal_pigpioI2cImuReading.h"
#include "hal_pigpio/hal_pigpioAnglesMsg.h"

struct Quaternions
{
    float w;
    float x;
    float y;
    float z;
};

struct Angles
{
    float phi;
    float theta;
    float psi;
};

class PigpioImu
{
private:
    int pigpioHandle;
    int32_t i2cHandle;
    Quaternions quaternions;
    Angles angles;
    bool isImuReady;
    ros::ServiceServer imuReadingService;
    ros::Timer readQuaternionsAndPublishAnglesTimer;
    ros::Publisher anglesPublisher;

public:
    PigpioImu(ros::NodeHandle *node, int pigpioHandle);
    ~PigpioImu() = default;
    void readQuaternions(void);
    void computeQuaternions(char (&data)[MPU6050_DMP_FIFO_QUAT_SIZE]);
    void publishAngles(void);
    void computeAngles(void);
    void readQuaternionsAndPublishAngles(const ros::TimerEvent &event);
    void resetFifo(void);
    uint16_t readFifoCount(void);
    bool isFifoOverflowed(void);
    bool i2cImuReading(hal_pigpio::hal_pigpioI2cImuReading::Request &req,
                       hal_pigpio::hal_pigpioI2cImuReading::Response &res);
};

#endif