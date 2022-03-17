#ifndef HAL_PIGPIO_IMU
#define HAL_PIGPIO_IMU

#include "ros/ros.h"
#include <vector>

// Pigpio library
#include <pigpiod_if2.h>

// Services and messages headers (generated)
#include "hal_pigpio/hal_pigpioImuReading.h"

#define I2C_BUFFER_MAX_BYTES 32

#define MPU6050_USER_CONTROL_REGISTER 0x6A
#define MPU6050_FIFO_RESET_BIT 2
#define MPU6050_INTERRUPT_STATUS_REGISTER 0x3A
#define MPU6050_FIFO_OVERFLOW 0x10
#define MPU6050_FIFO_COUNT_H_REGISTER 0x72
#define MPU6050_FIFO_COUNT_L_REGISTER 0x73
#define MPU6050_FIFO_REGISTER 0x74
#define MPU6050_DMP_FIFO_QUAT_SIZE 16

class PigpioImu
{
private:
    int pigpioHandle;
    int32_t i2cHandle;
    std::vector<uint32_t> quaternions;
    bool isImuReady;
    ros::ServiceServer imuReadingService;
    ros::Timer readAndPublishQuaternionsTimer;

public:
    PigpioImu(ros::NodeHandle *node, int pigpioHandle);
    ~PigpioImu() = default;
    void readImuData(void);
    void publishQuaternions(void);
    void readAndPublishQuaternions(const ros::TimerEvent &event);
    void resetFifo(void);
    bool imuReading(hal_pigpio::hal_pigpioImuReading::Request &req,
                    hal_pigpio::hal_pigpioImuReading::Response &res);
};

#endif