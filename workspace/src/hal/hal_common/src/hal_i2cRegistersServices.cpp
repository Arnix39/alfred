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

#include "hal_i2cRegistersServices.hpp"

int32_t getI2cHandle(imuGetHandleSyncClientNode_t imuGetHandleSyncClient)
{
  auto imuGetHandleRequest = std::make_shared<HalImuGetHandle_t::Request>();

  return imuGetHandleSyncClient.sendRequest(imuGetHandleRequest)->handle;
}

int16_t readByteFromRegister(
  i2cReadByteDataSyncClientNode_t i2cReadByteDataSyncClientNode, int32_t imuHandle,
  uint8_t registerToRead)
{
  int16_t byteRead = -1;

  auto i2cReadByteDataRequest = std::make_shared<HalPigpioI2cReadByteData_t::Request>();

  i2cReadByteDataRequest->handle = imuHandle;
  i2cReadByteDataRequest->device_register = registerToRead;

  auto response = i2cReadByteDataSyncClientNode.sendRequest(i2cReadByteDataRequest);

  if (response->has_succeeded) {
    byteRead = response->value;
  }

  return byteRead;
}

std::vector<uint8_t> readBlockFromRegister(
  i2cReadBlockDataSyncClientNode_t i2cReadBlockDataSyncClient, int32_t imuHandle,
  uint8_t registerToRead, uint8_t bytesToRead)
{
  std::vector<uint8_t> dataRead;

  auto i2cReadBlockDataRequest =
    std::make_shared<HalPigpioI2cReadBlockData_t::Request>();

  i2cReadBlockDataRequest->handle = imuHandle;
  i2cReadBlockDataRequest->device_register = registerToRead;
  i2cReadBlockDataRequest->length = bytesToRead;

  auto response = i2cReadBlockDataSyncClient.sendRequest(i2cReadBlockDataRequest);

  if (response->has_succeeded) {
    dataRead = response->data_block;
  }

  return dataRead;
}

bool writeBitInRegister(
  i2cReadByteDataSyncClientNode_t i2cReadByteDataSyncClientNode,
  i2cWriteByteDataSyncClientNode_t i2cWriteByteDataSyncClientNode, int32_t imuHandle,
  uint8_t registerToWrite, uint8_t bitToWrite, uint8_t valueOfBit)
{
  uint8_t registerValue;
  int16_t valueRead;
  uint8_t newRegisterValue;

  valueRead = readByteFromRegister(i2cReadByteDataSyncClientNode, imuHandle, registerToWrite);
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

  return writeByteInRegister(
    i2cWriteByteDataSyncClientNode,
    imuHandle,
    registerToWrite,
    newRegisterValue);
}

bool writeByteInRegister(
  i2cWriteByteDataSyncClientNode_t i2cWriteByteDataSyncClientNode, int32_t imuHandle,
  uint8_t registerToWrite, uint8_t value)
{
  auto i2cWriteByteDataRequest =
    std::make_shared<HalPigpioI2cWriteByteData_t::Request>();

  i2cWriteByteDataRequest->handle = imuHandle;
  i2cWriteByteDataRequest->device_register = registerToWrite;
  i2cWriteByteDataRequest->value = value;

  return i2cWriteByteDataSyncClientNode.sendRequest(i2cWriteByteDataRequest)->has_succeeded;
}

bool writeDataBlock(
  i2cWriteBlockDataSyncClientNode_t i2cWriteBlockDataSyncClientNode, int32_t imuHandle,
  uint8_t registerToWrite, std::vector<uint8_t> data)
{
  auto i2cWriteBlockDataRequest =
    std::make_shared<HalPigpioI2cWriteBlockData_t::Request>();

  i2cWriteBlockDataRequest->handle = imuHandle;
  i2cWriteBlockDataRequest->device_register = registerToWrite;
  i2cWriteBlockDataRequest->length = data.size();

  for (uint8_t index = 0; index < data.size(); ++index) {
    i2cWriteBlockDataRequest->data_block.push_back(data.at(index));
  }

  return i2cWriteBlockDataSyncClientNode.sendRequest(i2cWriteBlockDataRequest)->has_succeeded;
}
