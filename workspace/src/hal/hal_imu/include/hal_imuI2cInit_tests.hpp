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

#ifndef HAL_IMUI2CINIT_TESTS_HPP_
#define HAL_IMUI2CINIT_TESTS_HPP_

#include <memory>

#include "gtest/gtest.h"

#include "rclcpp/rclcpp.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"

#include "hal_imuI2cInit.hpp"
#include "pigpiod_if2.h" // NOLINT

using HalPigpioI2cOpen_t = hal_pigpio_interfaces::srv::HalPigpioI2cOpen;
using HalPigpioI2cClose_t = hal_pigpio_interfaces::srv::HalPigpioI2cClose;
using HalImuGetHandle_t = hal_imu_interfaces::srv::HalImuGetHandle;

class PigpioDummyNode : public rclcpp::Node
{
public:
  PigpioDummyNode();
  ~PigpioDummyNode();

  int piHandle;
  int32_t imuHandle;

  rclcpp::Service<HalPigpioI2cOpen_t>::SharedPtr i2cOpenService;
  rclcpp::Service<HalPigpioI2cClose_t>::SharedPtr i2cCloseService;

  void i2cOpen(
    const std::shared_ptr<HalPigpioI2cOpen_t::Request> request,
    std::shared_ptr<HalPigpioI2cOpen_t::Response> response);
  void i2cClose(
    const std::shared_ptr<HalPigpioI2cClose_t::Request> request,
    std::shared_ptr<HalPigpioI2cClose_t::Response> response);
};

class ImuI2cInitCheckerNode : public rclcpp::Node
{
public:
  ImuI2cInitCheckerNode();
  ~ImuI2cInitCheckerNode() = default;

  rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr changeStateClient;
  rclcpp::Client<HalImuGetHandle_t>::SharedPtr hal_imuGetHandleClient;

  void changeImuI2cInitNodeToState(std::uint8_t transition);
};

/* Test fixture */
class ImuI2cInitTest : public testing::Test
{
protected:
  std::shared_ptr<PigpioDummyNode> pigpioDummy;
  std::shared_ptr<ImuI2cInit> imuI2cInit;
  std::shared_ptr<ImuI2cInitCheckerNode> imuI2cInitChecker;
  rclcpp::executors::SingleThreadedExecutor executorImuI2cInit;
  rclcpp::executors::SingleThreadedExecutor executorPigpio;

  void SetUp()
  {
    pigpioDummy = std::make_shared<PigpioDummyNode>();
    imuI2cInitChecker = std::make_shared<ImuI2cInitCheckerNode>();
    imuI2cInit = std::make_shared<ImuI2cInit>();

    executorImuI2cInit.add_node(imuI2cInit->get_node_base_interface());
    executorImuI2cInit.add_node(imuI2cInitChecker);
    executorPigpio.add_node(pigpioDummy);

    imuI2cInitChecker->changeImuI2cInitNodeToState(
      lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    executorImuI2cInit.spin_some();
  }

  void TearDown()
  {
    imuI2cInitChecker->changeImuI2cInitNodeToState(
      lifecycle_msgs::msg::Transition::TRANSITION_INACTIVE_SHUTDOWN);
    executorImuI2cInit.spin_some();
    executorImuI2cInit.cancel();
    executorImuI2cInit.remove_node(imuI2cInit->get_node_base_interface());
    executorImuI2cInit.remove_node(imuI2cInitChecker);
    executorPigpio.remove_node(pigpioDummy);
    imuI2cInit.reset();
    pigpioDummy.reset();
    imuI2cInitChecker.reset();
  }
};

#endif  // HAL_IMUI2CINIT_TESTS_HPP_
