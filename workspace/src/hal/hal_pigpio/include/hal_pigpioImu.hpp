#ifndef HAL_PIGPIO_IMU
#define HAL_PIGPIO_IMU

#include <vector>
#define _USE_MATH_DEFINES
#include <cmath>

#include "rclcpp/rclcpp.hpp"

#include "hal_mpu6050.hpp"

// Pigpio library
#include <pigpiod_if2.h>

// Services and messages headers (generated)
#include "hal_pigpio/srv/hal_pigpio_i2c_imu_reading.hpp"
#include "hal_pigpio/msg/hal_pigpio_angles.hpp"

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
    std::shared_ptr<rclcpp::Node> halPigpioNode;
    int32_t i2cHandle;
    Quaternions quaternions;
    Angles angles;
    bool isImuReady;
    rclcpp::ServiceServer imuReadingService;
    rclcpp::Timer readQuaternionsAndPublishAnglesTimer;
    rclcpp::Publisher anglesPublisher;

public:
    PigpioImu(std::shared_ptr<rclcpp::Node> node, int pigpioHandle);
    ~PigpioImu() = default;
    void readQuaternions(void);
    void computeQuaternions(char (&data)[MPU6050_DMP_FIFO_QUAT_SIZE]);
    void publishAngles(void);
    void computeAngles(void);
    void readQuaternionsAndPublishAngles(const rclcpp::TimerEvent &event);
    void resetFifo(void);
    uint16_t readFifoCount(void);
    bool isFifoOverflowed(void);
    bool i2cImuReading(std::shared_ptr<hal_pigpio::srv::HalPigpioI2cImuReading::Request request,
                       std::shared_ptr<hal_pigpio::srv::HalPigpioI2cImuReading::Response response);
};

#endif