#ifndef HAL_PIGPIO_I2C
#define HAL_PIGPIO_I2C

#include "rclcpp/rclcpp.hpp"

// Pigpio library
#include <pigpiod_if2.h>

// Services and messages headers (generated)
#include "hal_pigpio/srv/hal_pigpio_i2c_open.hpp"
#include "hal_pigpio/srv/hal_pigpio_i2c_close.hpp"
#include "hal_pigpio/srv/hal_pigpio_i2c_read_byte_data.hpp"
#include "hal_pigpio/srv/hal_pigpio_i2c_read_word_data.hpp"
#include "hal_pigpio/srv/hal_pigpio_i2c_read_block_data.hpp"
#include "hal_pigpio/srv/hal_pigpio_i2c_write_byte_data.hpp"
#include "hal_pigpio/srv/hal_pigpio_i2c_write_word_data.hpp"
#include "hal_pigpio/srv/hal_pigpio_i2c_write_block_data.hpp"

#define I2C_BUFFER_MAX_BYTES 32

class PigpioI2c
{
private:
    int pigpioHandle;
    std::shared_ptr<rclcpp::Node> halPigpioNode;
    rclcpp::Service<hal_pigpio::srv::HalPigpioI2cOpen>::SharedPtr i2cOpenService;
    rclcpp::Service<hal_pigpio::srv::HalPigpioI2cClose>::SharedPtr i2cCloseService;
    rclcpp::Service<hal_pigpio::srv::HalPigpioI2cReadByteData>::SharedPtr i2cReadByteDataService;
    rclcpp::Service<hal_pigpio::srv::HalPigpioI2cReadWordData>::SharedPtr i2cReadWordDataService;
    rclcpp::Service<hal_pigpio::srv::HalPigpioI2cReadBlockData>::SharedPtr i2cReadBlockDataService;
    rclcpp::Service<hal_pigpio::srv::HalPigpioI2cWriteByteData>::SharedPtr i2cWriteByteDataService;
    rclcpp::Service<hal_pigpio::srv::HalPigpioI2cWriteWordData>::SharedPtr i2cWriteWordDataService;
    rclcpp::Service<hal_pigpio::srv::HalPigpioI2cWriteBlockData>::SharedPtr i2cWriteBlockDataService;

public:
    PigpioI2c(std::shared_ptr<rclcpp::Node> node, int pigpioHandle);
    ~PigpioI2c() = default;
    void i2cOpen(const std::shared_ptr<hal_pigpio::srv::HalPigpioI2cOpen::Request> request,
                 std::shared_ptr<hal_pigpio::srv::HalPigpioI2cOpen::Response> response);
    void i2cClose(const std::shared_ptr<hal_pigpio::srv::HalPigpioI2cClose::Request> request,
                  std::shared_ptr<hal_pigpio::srv::HalPigpioI2cClose::Response> response);
    void i2cReadByteData(const std::shared_ptr<hal_pigpio::srv::HalPigpioI2cReadByteData::Request> request,
                         std::shared_ptr<hal_pigpio::srv::HalPigpioI2cReadByteData::Response> response);
    void i2cReadWordData(const std::shared_ptr<hal_pigpio::srv::HalPigpioI2cReadWordData::Request> request,
                         std::shared_ptr<hal_pigpio::srv::HalPigpioI2cReadWordData::Response> response);
    void i2cReadBlockData(const std::shared_ptr<hal_pigpio::srv::HalPigpioI2cReadBlockData::Request> request,
                          std::shared_ptr<hal_pigpio::srv::HalPigpioI2cReadBlockData::Response> response);
    void i2cWriteByteData(const std::shared_ptr<hal_pigpio::srv::HalPigpioI2cWriteByteData::Request> request,
                          std::shared_ptr<hal_pigpio::srv::HalPigpioI2cWriteByteData::Response> response);
    void i2cWriteWordData(const std::shared_ptr<hal_pigpio::srv::HalPigpioI2cWriteWordData::Request> request,
                          std::shared_ptr<hal_pigpio::srv::HalPigpioI2cWriteWordData::Response> response);
    void i2cWriteBlockData(const std::shared_ptr<hal_pigpio::srv::HalPigpioI2cWriteBlockData::Request> request,
                           std::shared_ptr<hal_pigpio::srv::HalPigpioI2cWriteBlockData::Response> response);
};

#endif