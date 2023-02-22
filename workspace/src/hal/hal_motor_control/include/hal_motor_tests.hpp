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

#ifndef HAL_MOTOR_TESTS_HPP_
#define HAL_MOTOR_TESTS_HPP_

#include <memory>
#include <thread>

#include "gtest/gtest.h"

#include "rclcpp/rclcpp.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"

#include "hal_motor_control_commonDefinitions.hpp"
#include "hal_motor.hpp"
#include "pigpiod_if2.h" // NOLINT
#include "mock/raspberrypi_mock_utilities.hpp"

// Services and messages headers (generated)
#include "hal_pigpio_interfaces/srv/hal_pigpio_set_input_mode.hpp"
#include "hal_pigpio_interfaces/srv/hal_pigpio_set_output_mode.hpp"
#include "hal_pigpio_interfaces/srv/hal_pigpio_set_pwm_frequency.hpp"
#include "hal_pigpio_interfaces/srv/hal_pigpio_set_encoder_callback.hpp"

#define MOTOR_ID_1 0
#define GPIO_PWM_CHANNEL_A_M1 3
#define GPIO_PWM_CHANNEL_B_M1 4
#define GPIO_ENCODER_CHANNEL_A_M1 5
#define GPIO_ENCODER_CHANNEL_B_M1 6
#define INPUT 0
#define OUTPUT 1

class HalPigpioDummyNode : public rclcpp::Node
{
public:
  HalPigpioDummyNode();
  ~HalPigpioDummyNode();

  int piHandle;
  int32_t callbackId;

  rclcpp::Service<HalPigpioSetInputMode_t>::SharedPtr setInputModeService;
  rclcpp::Service<HalPigpioSetOutputMode_t>::SharedPtr setOutputModeService;
  rclcpp::Service<HalPigpioSetPwmDutycycle_t>::SharedPtr setPwmDutycycleService;
  rclcpp::Service<HalPigpioSetPwmFrequency_t>::SharedPtr setPwmFrequencyService;
  rclcpp::Service<HalPigpioSetEncoderCallback_t>::SharedPtr setEncoderCallbackService;
  rclcpp::Service<HalPigpioSetMotorDirection_t>::SharedPtr setMotorDirectionService;

  void setInputMode(
    const std::shared_ptr<HalPigpioSetInputMode_t::Request> request,
    std::shared_ptr<HalPigpioSetInputMode_t::Response> response);
  void setOutputMode(
    const std::shared_ptr<HalPigpioSetOutputMode_t::Request> request,
    std::shared_ptr<HalPigpioSetOutputMode_t::Response> response);
  void setPwmDutycycle(
    const std::shared_ptr<HalPigpioSetPwmDutycycle_t::Request> request,
    std::shared_ptr<HalPigpioSetPwmDutycycle_t::Response> response);
  void setPwmFrequency(
    const std::shared_ptr<HalPigpioSetPwmFrequency_t::Request> request,
    std::shared_ptr<HalPigpioSetPwmFrequency_t::Response> response);
  void setEncoderCallback(
    const std::shared_ptr<HalPigpioSetEncoderCallback_t::Request> request,
    std::shared_ptr<HalPigpioSetEncoderCallback_t::Response> response);
  void setMotorDirection(
    const std::shared_ptr<HalPigpioSetMotorDirection_t::Request> request,
    std::shared_ptr<HalPigpioSetMotorDirection_t::Response> response);
};

class MotorNode : public rclcpp::Node
{
public:
  MotorNode();

  Motor motorOk;
};

class MotorCheckerNode : public rclcpp::Node
{
public:
  MotorCheckerNode();
  ~MotorCheckerNode() = default;

  setOutputModeSyncClientNode_t setOutputModeClient;
  setInputModeSyncClientNode_t setInputModeClient;
  setEncoderCallbackSyncClientNode_t setEncoderCallbackClient;
  setPwmFrequencySyncClientNode_t setPwmFrequencyClient;

  rclcpp::Client<HalPigpioSetPwmDutycycle_t>::SharedPtr setPwmDutycycleClient;
  rclcpp::Client<HalPigpioSetMotorDirection_t>::SharedPtr setMotorDirectionClient;
};

/* Test fixture */
class MotorTest : public testing::Test
{
protected:
  std::shared_ptr<HalPigpioDummyNode> pigpioDummy;
  std::shared_ptr<MotorNode> motor;
  std::shared_ptr<MotorCheckerNode> motorChecker;
  rclcpp::executors::SingleThreadedExecutor executor;

  void SetUp()
  {
    pigpioDummy = std::make_shared<HalPigpioDummyNode>();
    motorChecker = std::make_shared<MotorCheckerNode>();
    motor = std::make_shared<MotorNode>();

    executor.add_node(motor);
    executor.add_node(motorChecker);
    executor.add_node(pigpioDummy);

    motorChecker->setInputModeClient.init("hal_pigpioSetInputMode");
    motorChecker->setOutputModeClient.init("hal_pigpioSetOutputMode");
    motorChecker->setEncoderCallbackClient.init("hal_pigpioSetEncoderCallback");
    motorChecker->setPwmFrequencyClient.init("hal_pigpioSetPwmFrequency");

    executor.spin_some();
  }

  void TearDown()
  {
    executor.cancel();
    executor.remove_node(motor);
    executor.remove_node(motorChecker);
    executor.remove_node(pigpioDummy);
    pigpioDummy.reset();
    motorChecker.reset();
    motor.reset();
  }
};

#endif  // HAL_MOTOR_TESTS_HPP_
