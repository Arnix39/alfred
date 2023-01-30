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

#ifndef HAL_IMU_TESTS_HPP_
#define HAL_IMU_TESTS_HPP_

#define BIAS_VALUE 0x1516
#define SENSOR_BIAS_MSB_REGISTER 0x11
#define SENSOR_BIAS_LSB_REGISTER 0x12

#include <memory>
#include <vector>

#include "gtest/gtest.h"

#include "rclcpp/rclcpp.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"

#include "hal_imu.hpp"
#include "mock/hal_i2cRegistersServicesMock.hpp"

class ImuCheckerNode : public rclcpp::Node
{
public:
  ImuCheckerNode();
  ~ImuCheckerNode() = default;

  rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr changeStateClient;
  i2cReadByteDataSyncClientNode_t i2cReadByteDataDummy;
  i2cReadBlockDataSyncClientNode_t i2cReadBlockDataDummy;
  i2cWriteByteDataSyncClientNode_t i2cWriteByteDataDummy;
  i2cWriteBlockDataSyncClientNode_t i2cWriteBlockDataDummy;
  imuGetHandleSyncClientNode_t imuGetHandleDummy;

  int32_t imuHandle;

  void changeImuNodeToState(std::uint8_t transition);
  void writeByte(uint8_t imuRegister, uint8_t value);
  int16_t readByte(uint8_t imuRegister);
  std::vector<uint8_t> readBlock(uint8_t imuRegister, uint8_t bytesToRead);
};

/* Test fixture */
class ImuTest : public testing::Test
{
protected:
  std::shared_ptr<Imu> imu;
  std::shared_ptr<ImuCheckerNode> imuChecker;
  rclcpp::executors::SingleThreadedExecutor executorImu;

  void SetUp()
  {
    imuChecker = std::make_shared<ImuCheckerNode>();
    imu = std::make_shared<Imu>();

    executorImu.add_node(imu->get_node_base_interface());
    executorImu.add_node(imuChecker);

    imuChecker->imuHandle = getI2cHandle(imuChecker->imuGetHandleDummy);
  }

  void TearDown()
  {
    executorImu.cancel();
    executorImu.remove_node(imu->get_node_base_interface());
    executorImu.remove_node(imuChecker);
    closePigpio(imuChecker->imuHandle);
    imu.reset();
    imuChecker.reset();
  }
};

#endif  // HAL_IMU_TESTS_HPP_
