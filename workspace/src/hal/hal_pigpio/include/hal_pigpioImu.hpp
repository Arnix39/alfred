#ifndef HAL_PIGPIO_IMU
#define HAL_PIGPIO_IMU

#include <chrono>
#include <vector>
#define _USE_MATH_DEFINES
#include <cmath>

#include "rclcpp/rclcpp.hpp"

#include "hal_mpu6050.hpp"

// Pigpio library
#include "pigpiod_if2.h"

// Services and messages headers (generated)
#include "hal_pigpio_interfaces/srv/hal_pigpio_i2c_imu_reading.hpp"
#include "hal_pigpio_interfaces/msg/hal_pigpio_angles.hpp"

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
    rclcpp::TimerBase::SharedPtr readQuaternionsAndPublishAnglesTimer;
    rclcpp::Service<hal_pigpio_interfaces::srv::HalPigpioI2cImuReading>::SharedPtr imuReadingService;
    rclcpp::Publisher<hal_pigpio_interfaces::msg::HalPigpioAngles>::SharedPtr anglesPublisher;

public:
    PigpioImu(std::shared_ptr<rclcpp::Node> node, int pigpioHandle);
    ~PigpioImu() = default;
    void readQuaternions(void);
    void computeQuaternions(char (&data)[MPU6050_DMP_FIFO_QUAT_SIZE]);
    void publishAngles(void);
    void computeAngles(void);
    void readQuaternionsAndPublishAngles(void);
    void resetFifo(void);
    uint16_t readFifoCount(void);
    bool isFifoOverflowed(void);
    void i2cImuReading(const std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioI2cImuReading::Request> request,
                       std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioI2cImuReading::Response> response);
};

#endif