#ifndef HAL_I2C_REGISTERS_SERVICES
#define HAL_I2C_REGISTERS_SERVICES

#include "hal_common.hpp"
#include "hal_mpu6050.hpp"

// Services and messages headers (generated)
#include "hal_pigpio_interfaces/srv/hal_pigpio_i2c_read_byte_data.hpp"
#include "hal_pigpio_interfaces/srv/hal_pigpio_i2c_write_byte_data.hpp"
#include "hal_pigpio_interfaces/srv/hal_pigpio_i2c_write_block_data.hpp"

using i2cReadByteDataFuture_t = rclcpp::Client<hal_pigpio_interfaces::srv::HalPigpioI2cReadByteData>::SharedFuture;
using i2cWriteByteDataFuture_t = rclcpp::Client<hal_pigpio_interfaces::srv::HalPigpioI2cWriteByteData>::SharedFuture;
using i2cWriteBlockDataFuture_t = rclcpp::Client<hal_pigpio_interfaces::srv::HalPigpioI2cWriteBlockData>::SharedFuture;

using i2cReadByteDataClient_t = rclcpp::Client<hal_pigpio_interfaces::srv::HalPigpioI2cReadByteData>::SharedPtr;
using i2cWriteByteDataClient_t = rclcpp::Client<hal_pigpio_interfaces::srv::HalPigpioI2cWriteByteData>::SharedPtr;
using i2cWriteBlockDataClient_t = rclcpp::Client<hal_pigpio_interfaces::srv::HalPigpioI2cWriteBlockData>::SharedPtr;

int16_t readByteFromRegister(i2cReadByteDataClient_t i2cReadByteDataClient, int32_t imuHandle, uint8_t registerToRead);
bool writeBitInRegister(i2cReadByteDataClient_t i2cReadByteDataClient, i2cWriteByteDataClient_t i2cWriteByteDataClient, int32_t imuHandle, uint8_t registerToWrite, uint8_t bitToWrite, uint8_t valueOfBit);
bool writeByteInRegister(i2cWriteByteDataClient_t i2cWriteByteDataClient, int32_t imuHandle, uint8_t registerToWrite, uint8_t value);
bool writeDataBlock(i2cWriteBlockDataClient_t i2cWriteBlockDataClient, int32_t imuHandle, uint8_t registerToWrite, std::vector<uint8_t> data);

#endif