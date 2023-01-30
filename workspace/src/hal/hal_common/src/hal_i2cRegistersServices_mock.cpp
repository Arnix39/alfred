// Copyright (c) 2023 Arnix Robotix
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

#include "hal_i2cRegistersServices.hpp"
#include "pigpiod_if2.h" // NOLINT

#define IMU_I2C_BUS 0x1
#define I2C_BUFFER_MAX_BYTES 32

static int piHandle = pigpio_start(NULL, NULL);

int32_t getI2cHandle(imuGetHandleSyncClientNode_t imuGetHandleSyncClient)
{
  (void)imuGetHandleSyncClient;

  return static_cast<int32_t>(i2c_open(piHandle, IMU_I2C_BUS, MPU6050_I2C_ADDRESS, 0));
}

int16_t readByteFromRegister(
  i2cReadByteDataSyncClientNode_t i2cReadByteDataSyncClientNode, int32_t imuHandle,
  uint8_t registerToRead)
{
  (void)i2cReadByteDataSyncClientNode;

  return static_cast<int16_t>(i2c_read_byte_data(piHandle, imuHandle, registerToRead));
}

bool writeBitInRegister(
  i2cReadByteDataSyncClientNode_t i2cReadByteDataSyncClientNode,
  i2cWriteByteDataSyncClientNode_t i2cWriteByteDataSyncClientNode, int32_t imuHandle,
  uint8_t registerToWrite, uint8_t bitToWrite, uint8_t valueOfBit)
{
  (void)i2cReadByteDataSyncClientNode;
  (void)i2cWriteByteDataSyncClientNode;

  uint8_t registerValue;
  int16_t valueRead;
  uint8_t newRegisterValue;

  valueRead = static_cast<int16_t>(i2c_read_byte_data(piHandle, imuHandle, registerToWrite));
  if (valueRead < 0) {
    return false;
  } else {
    registerValue = static_cast<uint8_t>(valueRead);
  }

  if (valueOfBit == 1) {
    newRegisterValue = registerValue | (1 << bitToWrite);
  } else {
    newRegisterValue = registerValue & ~(1 << bitToWrite);
  }

  if (i2c_write_byte_data(piHandle, imuHandle, registerToWrite, newRegisterValue) == 0) {
    return true;
  } else {
    return false;
  }
}

bool writeByteInRegister(
  i2cWriteByteDataSyncClientNode_t i2cWriteByteDataSyncClientNode, int32_t imuHandle,
  uint8_t registerToWrite, uint8_t value)
{
  (void)i2cWriteByteDataSyncClientNode;

  if (i2c_write_byte_data(piHandle, imuHandle, registerToWrite, value) == 0) {
    return true;
  } else {
    return false;
  }
}

bool writeDataBlock(
  i2cWriteBlockDataSyncClientNode_t i2cWriteBlockDataSyncClientNode, int32_t imuHandle,
  uint8_t registerToWrite, std::vector<uint8_t> data)
{
  (void)i2cWriteBlockDataSyncClientNode;

  char dataBlock[I2C_BUFFER_MAX_BYTES];

  for (uint8_t byte = 0; byte < data.size(); byte++) {
    dataBlock[byte] = static_cast<char>(data[byte]);
  }

  if (i2c_write_i2c_block_data(piHandle, imuHandle, registerToWrite, dataBlock, data.size()) == 0) {
    return true;
  } else {
    return false;
  }
}
