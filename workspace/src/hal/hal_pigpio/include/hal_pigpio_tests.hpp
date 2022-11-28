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

#ifndef HAL_PIGPIO_TESTS_HPP_
#define HAL_PIGPIO_TESTS_HPP_

#include <memory>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"

#include "hal_pigpio.hpp"

#define GOOD_GPIO GPIO2
#define BAD_GPIO GPIO1
#define MOTOR_ID_1 0
#define MOTOR_ID_2 1
#define FORWARD true

template<typename T>
bool hal_pigpioGpioSet(
  uint16_t gpio_id,
  std::shared_ptr<rclcpp::Client<T>> serviceClient,
  rclcpp::executors::SingleThreadedExecutor * executor)
{
  auto request = std::make_shared<typename T::Request>();

  request->gpio_id = gpio_id;

  auto future = serviceClient->async_send_request(request);

  executor->spin_until_future_complete(future);

  return future.get()->has_succeeded;
}

class PigioCheckerNode : public rclcpp::Node
{
  using HalPigpioSetInputMode_t = hal_pigpio_interfaces::srv::HalPigpioSetInputMode;
  using HalPigpioSetOutputMode_t = hal_pigpio_interfaces::srv::HalPigpioSetOutputMode;
  using HalPigpioGetMode_t = hal_pigpio_interfaces::srv::HalPigpioGetMode;
  using HalPigpioSetPullUp_t = hal_pigpio_interfaces::srv::HalPigpioSetPullUp;
  using HalPigpioSetPullDown_t = hal_pigpio_interfaces::srv::HalPigpioSetPullDown;
  using HalPigpioClearResistor_t = hal_pigpio_interfaces::srv::HalPigpioClearResistor;
  using HalPigpioSetPwmDutycycle_t = hal_pigpio_interfaces::srv::HalPigpioSetPwmDutycycle;
  using HalPigpioSetPwmFrequency_t = hal_pigpio_interfaces::srv::HalPigpioSetPwmFrequency;
  using HalPigpioSetGpioHigh_t = hal_pigpio_interfaces::srv::HalPigpioSetGpioHigh;
  using HalPigpioSetGpioLow_t = hal_pigpio_interfaces::srv::HalPigpioSetGpioLow;
  using HalPigpioSendTriggerPulse_t = hal_pigpio_interfaces::srv::HalPigpioSendTriggerPulse;
  using HalPigpioReadGpio_t = hal_pigpio_interfaces::srv::HalPigpioReadGpio;
  using HalPigpioSetCallback_t = hal_pigpio_interfaces::srv::HalPigpioSetCallback;
  using HalPigpioSetEncoderCallback_t = hal_pigpio_interfaces::srv::HalPigpioSetEncoderCallback;
  using HalPigpioSetMotorDirection_t = hal_pigpio_interfaces::srv::HalPigpioSetMotorDirection;

public:
  PigioCheckerNode()
  : rclcpp::Node("hal_pigpio_checker_node"),
    changeStateClient(this->create_client<lifecycle_msgs::srv::ChangeState>(
        "hal_pigpio_node/change_state")),
    setInputModeClient(this->create_client<HalPigpioSetInputMode_t>("hal_pigpioSetInputMode")),
    setOutputModeClient(this->create_client<HalPigpioSetOutputMode_t>("hal_pigpioSetOutputMode")),
    getModeClient(this->create_client<HalPigpioGetMode_t>("hal_pigpioGetMode")),
    setPullUpClient(this->create_client<HalPigpioSetPullUp_t>("hal_pigpioSetPullUp")),
    setPullDownClient(this->create_client<HalPigpioSetPullDown_t>("hal_pigpioSetPullDown")),
    clearResistorClient(this->create_client<HalPigpioClearResistor_t>("hal_pigpioClearResistor")),
    setPwmDutycycleClient(this->create_client<HalPigpioSetPwmDutycycle_t>(
        "hal_pigpioSetPwmDutycycle")),
    setPwmFrequencyClient(this->create_client<HalPigpioSetPwmFrequency_t>(
        "hal_pigpioSetPwmFrequency")),
    setGpioHighClient(this->create_client<HalPigpioSetGpioHigh_t>("hal_pigpioSetGpioHigh")),
    setGpioLowClient(this->create_client<HalPigpioSetGpioLow_t>("hal_pigpioSetGpioLow")),
    sendTriggerPulseClient(this->create_client<HalPigpioSendTriggerPulse_t>(
        "hal_pigpioSendTriggerPulse")),
    readGpioClient(this->create_client<HalPigpioReadGpio_t>("hal_pigpioReadGpio")),
    setCallbackClient(this->create_client<HalPigpioSetCallback_t>("hal_pigpioSetCallback")),
    setEncoderCallbackClient(this->create_client<HalPigpioSetEncoderCallback_t>(
        "hal_pigpioSetEncoderCallback")),
    setMotorDirectionClient(this->create_client<HalPigpioSetMotorDirection_t>(
        "hal_pigpioSetMotorDirection"))
  {
  }
  ~PigioCheckerNode() = default;

  rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr changeStateClient;
  rclcpp::Client<HalPigpioSetInputMode_t>::SharedPtr setInputModeClient;
  rclcpp::Client<HalPigpioSetOutputMode_t>::SharedPtr setOutputModeClient;
  rclcpp::Client<HalPigpioGetMode_t>::SharedPtr getModeClient;
  rclcpp::Client<HalPigpioSetPullUp_t>::SharedPtr setPullUpClient;
  rclcpp::Client<HalPigpioSetPullDown_t>::SharedPtr setPullDownClient;
  rclcpp::Client<HalPigpioClearResistor_t>::SharedPtr clearResistorClient;
  rclcpp::Client<HalPigpioSetPwmDutycycle_t>::SharedPtr setPwmDutycycleClient;
  rclcpp::Client<HalPigpioSetPwmFrequency_t>::SharedPtr setPwmFrequencyClient;
  rclcpp::Client<HalPigpioSetGpioHigh_t>::SharedPtr setGpioHighClient;
  rclcpp::Client<HalPigpioSetGpioLow_t>::SharedPtr setGpioLowClient;
  rclcpp::Client<HalPigpioSendTriggerPulse_t>::SharedPtr sendTriggerPulseClient;
  rclcpp::Client<HalPigpioReadGpio_t>::SharedPtr readGpioClient;
  rclcpp::Client<HalPigpioSetCallback_t>::SharedPtr setCallbackClient;
  rclcpp::Client<HalPigpioSetEncoderCallback_t>::SharedPtr setEncoderCallbackClient;
  rclcpp::Client<HalPigpioSetMotorDirection_t>::SharedPtr setMotorDirectionClient;

  void changePigpioNodeToState(std::uint8_t transition)
  {
    auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    request->transition.id = transition;
    auto result = changeStateClient->async_send_request(request);
  }

  bool setPwmDutycycle(
    uint16_t gpio_id,
    uint8_t dutycycle,
    rclcpp::executors::SingleThreadedExecutor * executor)
  {
    auto request = std::make_shared<HalPigpioSetPwmDutycycle_t::Request>();

    request->gpio_id = gpio_id;
    request->dutycycle = dutycycle;

    auto future = setPwmDutycycleClient->async_send_request(request);

    executor->spin_until_future_complete(future);

    return future.get()->has_succeeded;
  }

  bool setPwmFrequency(
    uint16_t gpio_id,
    uint16_t frequency,
    rclcpp::executors::SingleThreadedExecutor * executor)
  {
    auto request = std::make_shared<HalPigpioSetPwmFrequency_t::Request>();

    request->gpio_id = gpio_id;
    request->frequency = frequency;

    auto future = setPwmFrequencyClient->async_send_request(request);

    executor->spin_until_future_complete(future);

    return future.get()->has_succeeded;
  }

  bool sendTriggerPulse(
    uint16_t gpio_id,
    uint16_t pulse_length_in_us,
    rclcpp::executors::SingleThreadedExecutor * executor)
  {
    auto request = std::make_shared<HalPigpioSendTriggerPulse_t::Request>();

    request->gpio_id = gpio_id;
    request->pulse_length_in_us = pulse_length_in_us;

    auto future = sendTriggerPulseClient->async_send_request(request);

    executor->spin_until_future_complete(future);

    return future.get()->has_succeeded;
  }

  bool readGpioAndCheckLevel(
    uint16_t gpio_id,
    uint8_t gpio_level,
    rclcpp::executors::SingleThreadedExecutor * executor)
  {
    auto request = std::make_shared<HalPigpioReadGpio_t::Request>();

    request->gpio_id = gpio_id;

    auto future = readGpioClient->async_send_request(request);

    executor->spin_until_future_complete(future);

    auto response = future.get();

    if (response->has_succeeded && response->level == gpio_level) {
      return true;
    }
    return false;
  }

  bool setCallback(
    uint16_t gpio_id,
    uint8_t edge_change_type,
    rclcpp::executors::SingleThreadedExecutor * executor)
  {
    auto request = std::make_shared<HalPigpioSetCallback_t::Request>();

    request->gpio_id = gpio_id;
    request->edge_change_type = edge_change_type;

    auto future = setCallbackClient->async_send_request(request);

    executor->spin_until_future_complete(future);

    return future.get()->has_succeeded;
  }

  bool setEncoderCallback(
    uint16_t gpio_id,
    uint8_t edge_change_type,
    uint8_t motor_id,
    rclcpp::executors::SingleThreadedExecutor * executor)
  {
    auto request = std::make_shared<HalPigpioSetEncoderCallback_t::Request>();

    request->gpio_id = gpio_id;
    request->edge_change_type = edge_change_type;
    request->motor_id = motor_id;

    auto future = setEncoderCallbackClient->async_send_request(request);

    executor->spin_until_future_complete(future);

    return future.get()->has_succeeded;
  }

  bool setMotorDirection(
    bool is_direction_forward,
    uint8_t motor_id,
    rclcpp::executors::SingleThreadedExecutor * executor)
  {
    auto request = std::make_shared<HalPigpioSetMotorDirection_t::Request>();

    request->is_direction_forward = is_direction_forward;
    request->motor_id = motor_id;

    auto future = setMotorDirectionClient->async_send_request(request);

    executor->spin_until_future_complete(future);

    return future.get()->has_succeeded;
  }
};

/* Test fixture */
class PigpioTest : public testing::Test
{
protected:
  std::shared_ptr<Pigpio> pigpio;
  std::shared_ptr<PigioCheckerNode> pigioChecker;
  rclcpp::executors::SingleThreadedExecutor executor;

  void SetUp()
  {
    pigioChecker = std::make_shared<PigioCheckerNode>();
    pigpio = std::make_shared<Pigpio>();

    executor.add_node(pigpio->get_node_base_interface());
    executor.add_node(pigioChecker);

    pigioChecker->changePigpioNodeToState(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    executor.spin_some();
    pigioChecker->changePigpioNodeToState(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
    executor.spin_some();
  }

  void TearDown()
  {
    executor.cancel();
    executor.remove_node(pigpio->get_node_base_interface());
    executor.remove_node(pigioChecker);
    pigpio.reset();
    pigioChecker.reset();
  }
};

#endif  // HAL_PIGPIO_TESTS_HPP_
