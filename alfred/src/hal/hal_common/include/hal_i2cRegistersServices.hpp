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

#ifndef HAL_I2CREGISTERSSERVICES_HPP_
#define HAL_I2CREGISTERSSERVICES_HPP_

#include <vector>
#include <chrono>
#include <memory>
#include <string>

#include "common.hpp"
#include "mpu6050.hpp"

// Services and messages headers (generated)
#include "hal_pigpio_interfaces/srv/hal_pigpio_i2c_read_byte_data.hpp"
#include "hal_pigpio_interfaces/srv/hal_pigpio_i2c_read_block_data.hpp"
#include "hal_pigpio_interfaces/srv/hal_pigpio_i2c_write_byte_data.hpp"
#include "hal_pigpio_interfaces/srv/hal_pigpio_i2c_write_block_data.hpp"
#include "hal_imu_interfaces/srv/hal_imu_get_handle.hpp"

using HalImuGetHandle_t = hal_imu_interfaces::srv::HalImuGetHandle;
using HalPigpioI2cReadByteData_t = hal_pigpio_interfaces::srv::HalPigpioI2cReadByteData;
using HalPigpioI2cReadBlockData_t = hal_pigpio_interfaces::srv::HalPigpioI2cReadBlockData;
using HalPigpioI2cWriteByteData_t = hal_pigpio_interfaces::srv::HalPigpioI2cWriteByteData;
using HalPigpioI2cWriteBlockData_t = hal_pigpio_interfaces::srv::HalPigpioI2cWriteBlockData;

using i2cReadByteDataSyncClientNode_t = ServiceNodeSync<HalPigpioI2cReadByteData_t>;
using i2cReadBlockDataSyncClientNode_t = ServiceNodeSync<HalPigpioI2cReadBlockData_t>;
using i2cWriteByteDataSyncClientNode_t = ServiceNodeSync<HalPigpioI2cWriteByteData_t>;
using i2cWriteBlockDataSyncClientNode_t = ServiceNodeSync<HalPigpioI2cWriteBlockData_t>;

using imuGetHandleSyncClientNode_t = ServiceNodeSync<HalImuGetHandle_t>;

namespace hal
{
namespace common
{

int32_t getI2cHandle(imuGetHandleSyncClientNode_t imuGetHandleSyncClientNode);

int16_t readByteFromRegister(
  i2cReadByteDataSyncClientNode_t i2cReadByteDataSyncClientNode, int32_t imuHandle,
  uint8_t registerToRead);

std::vector<uint8_t> readBlockFromRegister(
  i2cReadBlockDataSyncClientNode_t i2cReadBlockDataSyncClient, int32_t imuHandle,
  uint8_t registerToRead, uint8_t bytesToRead);

bool writeBitInRegister(
  i2cReadByteDataSyncClientNode_t i2cReadByteDataSyncClientNode,
  i2cWriteByteDataSyncClientNode_t i2cWriteByteDataSyncClientNode, int32_t imuHandle,
  uint8_t registerToWrite, uint8_t bitToWrite, uint8_t valueOfBit);

bool writeByteInRegister(
  i2cWriteByteDataSyncClientNode_t i2cWriteByteDataSyncClientNode, int32_t imuHandle,
  uint8_t registerToWrite, uint8_t value);

bool writeDataBlock(
  i2cWriteBlockDataSyncClientNode_t i2cWriteBlockDataSyncClientNode, int32_t imuHandle,
  uint8_t registerToWrite, std::vector<uint8_t> data);

}  // namespace common
}  // namespace hal

#endif  // HAL_I2CREGISTERSSERVICES_HPP_
