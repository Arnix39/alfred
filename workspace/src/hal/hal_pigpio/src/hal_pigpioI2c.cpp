#include "hal_pigpio.hpp"

void Pigpio::i2cOpen(
  const std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioI2cOpen::Request> request,
  std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioI2cOpen::Response> response)
{
  response->handle = i2c_open(pigpioHandle, request->bus, request->address, 0);
  if (response->handle >= 0) {
    response->has_succeeded = true;
    RCLCPP_INFO(
      get_logger(), "I2C bus %u open for device %u with handle %u.", request->bus, request->address,
      response->handle);
  } else {
    response->has_succeeded = false;
    RCLCPP_ERROR(
      get_logger(), "Failed to open I2C bus %u for device %u.", request->bus, request->address);
  }
}

void Pigpio::i2cClose(
  const std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioI2cClose::Request> request,
  std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioI2cClose::Response> response)
{
  if (i2c_close(pigpioHandle, request->handle) == 0) {
    response->has_succeeded = true;
    RCLCPP_INFO(get_logger(), "I2C device with handle %u closed.", request->handle);
  } else {
    response->has_succeeded = false;
    RCLCPP_ERROR(get_logger(), "Failed to close I2C device with handle %u.", request->handle);
  }
}

void Pigpio::i2cReadByteData(
  const std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioI2cReadByteData::Request> request,
  std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioI2cReadByteData::Response> response)
{
  int result = i2c_read_byte_data(pigpioHandle, request->handle, request->device_register);
  if (result >= 0) {
    response->value = static_cast<uint8_t>(result);
    response->has_succeeded = true;
  } else {
    response->value = 0;
    response->has_succeeded = false;
    RCLCPP_ERROR(
      get_logger(), "Failed to read register %u on device with handle %u.", request->device_register,
      request->handle);
  }
}

void Pigpio::i2cReadWordData(
  const std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioI2cReadWordData::Request> request,
  std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioI2cReadWordData::Response> response)
{
  int result = i2c_read_word_data(pigpioHandle, request->handle, request->device_register);
  if (result >= 0) {
    response->value = static_cast<uint16_t>(result);
    response->has_succeeded = true;
  } else {
    response->value = 0;
    response->has_succeeded = false;
    RCLCPP_ERROR(
      get_logger(), "Failed to read register %u on device with handle %u.", request->device_register,
      request->handle);
  }
}

void Pigpio::i2cReadBlockData(
  const std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioI2cReadBlockData::Request> request,
  std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioI2cReadBlockData::Response> response)
{
  char buffer[I2C_BUFFER_MAX_BYTES];
  int result = i2c_read_i2c_block_data(
    pigpioHandle, request->handle, request->device_register,
    buffer, request->length);
  if (result > 0) {
    response->has_succeeded = true;

    for (uint8_t index = 0; index < result; index++) {
      response->data_block.push_back(buffer[index]);
    }

    RCLCPP_INFO(
      get_logger(), "Successfuly read data block from register %u on device with handle %u.", request->device_register,
      request->handle);
  } else {
    response->has_succeeded = false;
    RCLCPP_ERROR(
      get_logger(), "Failed to read register %u on device with handle %u.", request->device_register,
      request->handle);
  }
}

void Pigpio::i2cWriteByteData(
  const std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioI2cWriteByteData::Request> request,
  std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioI2cWriteByteData::Response> response)
{
  if (i2c_write_byte_data(
      pigpioHandle, request->handle, request->device_register,
      request->value) == 0)
  {
    response->has_succeeded = true;
  } else {
    response->has_succeeded = false;
    RCLCPP_ERROR(
      get_logger(), "Failed to write register %u on device with handle %u.", request->device_register,
      request->handle);
  }
}

void Pigpio::i2cWriteWordData(
  const std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioI2cWriteWordData::Request> request,
  std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioI2cWriteWordData::Response> response)
{
  if (i2c_write_word_data(
      pigpioHandle, request->handle, request->device_register,
      request->value) == 0)
  {
    response->has_succeeded = true;
  } else {
    response->has_succeeded = false;
    RCLCPP_ERROR(
      get_logger(), "Failed to write register %u on device with handle %u.", request->device_register,
      request->handle);
  }
}

void Pigpio::i2cWriteBlockData(
  const std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioI2cWriteBlockData::Request> request,
  std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioI2cWriteBlockData::Response> response)
{
  char dataBlock[I2C_BUFFER_MAX_BYTES];

  for (uint8_t byte = 0; byte < request->length; byte++) {
    dataBlock[byte] = (char)(request->data_block[byte]);
  }

  if (i2c_write_i2c_block_data(
      pigpioHandle, request->handle, request->device_register, dataBlock,
      request->length) == 0)
  {
    response->has_succeeded = true;
  } else {
    response->has_succeeded = false;
    RCLCPP_ERROR(
      get_logger(), "Failed to write data block in register %u on device with handle %u.", request->device_register,
      request->handle);
  }
}
