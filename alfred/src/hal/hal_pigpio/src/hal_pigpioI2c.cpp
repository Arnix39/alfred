// Copyright (c) 2022 Arnix Robotix
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "hal_pigpio.hpp"

namespace hal
{
namespace pigpio
{

void Pigpio::i2cOpen(
  const std::shared_ptr<HalPigpioI2cOpen_t::Request> request,
  std::shared_ptr<HalPigpioI2cOpen_t::Response> response)
{
  response->handle = i2c_open(pigpioHandle, request->bus, request->address, 0);
  if (response->handle >= 0) {
    response->has_succeeded = true;
    RCLCPP_INFO(
      get_logger(), "I2C bus %u open for device %u with handle %u.",
      request->bus, request->address, response->handle);
  } else {
    response->has_succeeded = false;
    RCLCPP_ERROR(
      get_logger(), "Failed to open I2C bus %u for device %u.",
      request->bus, request->address);
  }
}

void Pigpio::i2cClose(
  const std::shared_ptr<HalPigpioI2cClose_t::Request> request,
  std::shared_ptr<HalPigpioI2cClose_t::Response> response)
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
  const std::shared_ptr<HalPigpioI2cReadByteData_t::Request> request,
  std::shared_ptr<HalPigpioI2cReadByteData_t::Response> response)
{
  int result = i2c_read_byte_data(pigpioHandle, request->handle, request->device_register);
  if (result >= 0) {
    response->value = static_cast<uint8_t>(result);
    response->has_succeeded = true;
  } else {
    response->value = 0;
    response->has_succeeded = false;
    RCLCPP_ERROR(
      get_logger(), "Failed to read register %u on device with handle %u.",
      request->device_register, request->handle);
  }
}

void Pigpio::i2cReadWordData(
  const std::shared_ptr<HalPigpioI2cReadWordData_t::Request> request,
  std::shared_ptr<HalPigpioI2cReadWordData_t::Response> response)
{
  int result = i2c_read_word_data(pigpioHandle, request->handle, request->device_register);
  if (result >= 0) {
    response->value = static_cast<uint16_t>(result);
    response->has_succeeded = true;
  } else {
    response->value = 0;
    response->has_succeeded = false;
    RCLCPP_ERROR(
      get_logger(), "Failed to read register %u on device with handle %u.",
      request->device_register, request->handle);
  }
}

void Pigpio::i2cReadBlockData(
  const std::shared_ptr<HalPigpioI2cReadBlockData_t::Request> request,
  std::shared_ptr<HalPigpioI2cReadBlockData_t::Response> response)
{
  char buffer[I2C_BUFFER_MAX_BYTES];
  int result = i2c_read_i2c_block_data(
    pigpioHandle, request->handle, request->device_register, buffer, request->length);
  if (result > 0) {
    response->has_succeeded = true;

    for (uint8_t index = 0; index < result; index++) {
      response->data_block.push_back(buffer[index]);
    }

    RCLCPP_INFO(
      get_logger(), "Successfuly read data block from register %u on device with handle %u.",
      request->device_register, request->handle);
  } else {
    response->has_succeeded = false;
    RCLCPP_ERROR(
      get_logger(), "Failed to read register %u on device with handle %u.",
      request->device_register, request->handle);
  }
}

void Pigpio::i2cWriteByteData(
  const std::shared_ptr<HalPigpioI2cWriteByteData_t::Request> request,
  std::shared_ptr<HalPigpioI2cWriteByteData_t::Response> response)
{
  if (i2c_write_byte_data(
      pigpioHandle, request->handle, request->device_register, request->value) == 0)
  {
    response->has_succeeded = true;
  } else {
    response->has_succeeded = false;
    RCLCPP_ERROR(
      get_logger(), "Failed to write register %u on device with handle %u.",
      request->device_register, request->handle);
  }
}

void Pigpio::i2cWriteWordData(
  const std::shared_ptr<HalPigpioI2cWriteWordData_t::Request> request,
  std::shared_ptr<HalPigpioI2cWriteWordData_t::Response> response)
{
  if (i2c_write_word_data(
      pigpioHandle, request->handle, request->device_register, request->value) == 0)
  {
    response->has_succeeded = true;
  } else {
    response->has_succeeded = false;
    RCLCPP_ERROR(
      get_logger(), "Failed to write register %u on device with handle %u.",
      request->device_register, request->handle);
  }
}

void Pigpio::i2cWriteBlockData(
  const std::shared_ptr<HalPigpioI2cWriteBlockData_t::Request> request,
  std::shared_ptr<HalPigpioI2cWriteBlockData_t::Response> response)
{
  char dataBlock[I2C_BUFFER_MAX_BYTES];

  for (uint8_t byte = 0; byte < request->length; byte++) {
    dataBlock[byte] = static_cast<char>(request->data_block[byte]);
  }

  if (i2c_write_i2c_block_data(
      pigpioHandle, request->handle, request->device_register, dataBlock, request->length) == 0)
  {
    response->has_succeeded = true;
  } else {
    response->has_succeeded = false;
    RCLCPP_ERROR(
      get_logger(), "Failed to write data block in register %u on device with handle %u.",
      request->device_register, request->handle);
  }
}

}  // namespace pigpio
}  // namespace hal
