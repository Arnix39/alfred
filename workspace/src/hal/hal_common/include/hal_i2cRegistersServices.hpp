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

template<class ServiceT>
class ServiceNodeSync
{
  typedef typename ServiceT::Request RequestT;
  typedef typename ServiceT::Response ResponseT;

public:
  explicit ServiceNodeSync(std::string name)
  : node(std::make_shared<rclcpp::Node>(name)) {}

  ~ServiceNodeSync()
  {
    node.reset();
    client.reset();
  }

  void init(std::string service)
  {
    client = node->create_client<ServiceT>(service);
    client->wait_for_service();
  }

  ResponseT sendRequest(const RequestT & req)
  {
    return sendRequest(std::make_shared<RequestT>(req));
  }

  std::shared_ptr<ResponseT> sendRequest(const std::shared_ptr<RequestT> & req_ptr)
  {
    std::shared_ptr<ResponseT> response;
    auto result = client->async_send_request(req_ptr);
    auto status = rclcpp::spin_until_future_complete(
      node, result, std::chrono::duration<int64_t, std::milli>(1000));

    if (status == rclcpp::FutureReturnCode::SUCCESS) {
      response = result.get();
    }

    return response;
  }

protected:
  rclcpp::Node::SharedPtr node;
  typename rclcpp::Client<ServiceT>::SharedPtr client;
};

using i2cReadByteDataSyncClientNode_t = ServiceNodeSync<HalPigpioI2cReadByteData_t>;
using i2cReadBlockDataSyncClientNode_t = ServiceNodeSync<HalPigpioI2cReadBlockData_t>;
using i2cWriteByteDataSyncClientNode_t = ServiceNodeSync<HalPigpioI2cWriteByteData_t>;
using i2cWriteBlockDataSyncClientNode_t = ServiceNodeSync<HalPigpioI2cWriteBlockData_t>;

using imuGetHandleSyncClientNode_t = ServiceNodeSync<HalImuGetHandle_t>;

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

#endif  // HAL_I2CREGISTERSSERVICES_HPP_
