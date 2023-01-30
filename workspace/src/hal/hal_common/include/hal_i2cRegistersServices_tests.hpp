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

#ifndef HAL_I2CREGISTERSSERVICES_TESTS_HPP_
#define HAL_I2CREGISTERSSERVICES_TESTS_HPP_

#include <memory>
#include <thread>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"

#include "hal_i2cRegistersServices.hpp"
#include "pigpiod_if2.h" // NOLINT

// Services and messages headers (generated)
#include "hal_pigpio_interfaces/srv/hal_pigpio_i2c_open.hpp"
#include "hal_pigpio_interfaces/srv/hal_pigpio_i2c_read_byte_data.hpp"
#include "hal_pigpio_interfaces/srv/hal_pigpio_i2c_read_block_data.hpp"
#include "hal_pigpio_interfaces/srv/hal_pigpio_i2c_write_byte_data.hpp"

#define I2C_GOOD_ADDRESS MPU6050_I2C_ADDRESS
#define I2C_GOOD_BUS_1 0
#define I2C_GOOD_HANDLE 0
#define I2C_BAD_REGISTER 0x30
#define I2C_GOOD_REGISTER 0x10
#define I2C_GOOD_REGISTER_2 0x20
#define I2C_BUFFER_MAX_BYTES 32
#define IMU_GOOD_HANDLE 1
#define DATA_BLOCK_SIZE 5

using HalPigpioI2cOpen_t = hal_pigpio_interfaces::srv::HalPigpioI2cOpen;
using HalImuGetHandle_t = hal_imu_interfaces::srv::HalImuGetHandle;
using HalPigpioI2cReadByteData_t = hal_pigpio_interfaces::srv::HalPigpioI2cReadByteData;
using HalPigpioI2cReadBlockData_t = hal_pigpio_interfaces::srv::HalPigpioI2cReadBlockData;
using HalPigpioI2cWriteByteData_t = hal_pigpio_interfaces::srv::HalPigpioI2cWriteByteData;
using HalPigpioI2cWriteBlockData_t = hal_pigpio_interfaces::srv::HalPigpioI2cWriteBlockData;

class HalDummyNode : public rclcpp::Node
{
public:
  HalDummyNode();
  ~HalDummyNode();

  int piHandle;

  rclcpp::Service<HalImuGetHandle_t>::SharedPtr imuGetHandleService;
  rclcpp::Service<HalPigpioI2cReadByteData_t>::SharedPtr i2cReadByteDataService;
  rclcpp::Service<HalPigpioI2cReadBlockData_t>::SharedPtr i2cReadBlockDataService;
  rclcpp::Service<HalPigpioI2cWriteByteData_t>::SharedPtr i2cWriteByteDataService;
  rclcpp::Service<HalPigpioI2cWriteBlockData_t>::SharedPtr i2cWriteBlockDataService;

  void getHandle(
    const std::shared_ptr<HalImuGetHandle_t::Request> request,
    std::shared_ptr<HalImuGetHandle_t::Response> response);
  void i2cReadByteData(
    const std::shared_ptr<HalPigpioI2cReadByteData_t::Request> request,
    std::shared_ptr<HalPigpioI2cReadByteData_t::Response> response);
  void i2cReadBlockData(
    const std::shared_ptr<HalPigpioI2cReadBlockData_t::Request> request,
    std::shared_ptr<HalPigpioI2cReadBlockData_t::Response> response);
  void i2cWriteByteData(
    const std::shared_ptr<HalPigpioI2cWriteByteData_t::Request> request,
    std::shared_ptr<HalPigpioI2cWriteByteData_t::Response> response);
  void i2cWriteBlockData(
    const std::shared_ptr<HalPigpioI2cWriteBlockData_t::Request> request,
    std::shared_ptr<HalPigpioI2cWriteBlockData_t::Response> response);
};

class I2cRegistersServicesCheckerNode : public rclcpp::Node
{
public:
  I2cRegistersServicesCheckerNode();
  ~I2cRegistersServicesCheckerNode() = default;

  i2cReadByteDataSyncClientNode_t i2cReadByteDataSyncClient;
  i2cReadBlockDataSyncClientNode_t i2cReadBlockDataSyncClient;
  i2cWriteByteDataSyncClientNode_t i2cWriteByteDataSyncClient;
  i2cWriteBlockDataSyncClientNode_t i2cWriteBlockDataSyncClient;
  imuGetHandleSyncClientNode_t imuGetHandleSyncClient;
};

/* Test fixture */
class I2cRegistersServicesTest : public testing::Test
{
protected:
  std::shared_ptr<I2cRegistersServicesCheckerNode> i2cRegistersServicesChecker;
  std::shared_ptr<HalDummyNode> halDummyNode;
  rclcpp::executors::SingleThreadedExecutor executor;

  void SetUp()
  {
    i2cRegistersServicesChecker = std::make_shared<I2cRegistersServicesCheckerNode>();
    halDummyNode = std::make_shared<HalDummyNode>();
    executor.add_node(halDummyNode);
    executor.add_node(i2cRegistersServicesChecker);

    i2cRegistersServicesChecker->imuGetHandleSyncClient.init("hal_imuGetHandle");
    i2cRegistersServicesChecker->i2cReadByteDataSyncClient.init("hal_pigpioI2cReadByteData");
    i2cRegistersServicesChecker->i2cReadBlockDataSyncClient.init("hal_pigpioI2cReadBlockData");
    i2cRegistersServicesChecker->i2cWriteByteDataSyncClient.init("hal_pigpioI2cWriteByteData");
    i2cRegistersServicesChecker->i2cWriteBlockDataSyncClient.init("hal_pigpioI2cWriteBlockData");

    executor.spin_some();
  }

  void TearDown()
  {
    executor.cancel();
    executor.remove_node(i2cRegistersServicesChecker);
    executor.remove_node(halDummyNode);
    halDummyNode.reset();
    i2cRegistersServicesChecker.reset();
  }
};

#endif  // HAL_I2CREGISTERSSERVICES_TESTS_HPP_
