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

#define GOOD_GPIO 1
#define BAD_GPIO 41

template<typename T>
bool hal_pigpioGpioSet(
  uint8_t gpio_id,
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

private:
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
        "hal_pigpioSendTriggerPulse"))
  {
  }
  ~PigioCheckerNode() = default;

  void changePigpioNodeToState(std::uint8_t transition)
  {
    auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    request->transition.id = transition;
    auto result = changeStateClient->async_send_request(request);
  }

  rclcpp::Client<HalPigpioSetInputMode_t>::SharedPtr getSetInputModeClient(void)
  {
    return setInputModeClient;
  }
  rclcpp::Client<HalPigpioSetOutputMode_t>::SharedPtr getSetOutputModeClient(void)
  {
    return setOutputModeClient;
  }
  rclcpp::Client<HalPigpioGetMode_t>::SharedPtr getGetModeClient(void)
  {
    return getModeClient;
  }
  rclcpp::Client<HalPigpioSetPullUp_t>::SharedPtr getSetPullUpClient(void)
  {
    return setPullUpClient;
  }
  rclcpp::Client<HalPigpioSetPullDown_t>::SharedPtr getSetPullDownClient(void)
  {
    return setPullDownClient;
  }
  rclcpp::Client<HalPigpioClearResistor_t>::SharedPtr getClearResistorClient(void)
  {
    return clearResistorClient;
  }
  rclcpp::Client<HalPigpioSetPwmDutycycle_t>::SharedPtr getSetPwmDutycycleClient(void)
  {
    return setPwmDutycycleClient;
  }
  rclcpp::Client<HalPigpioSetPwmFrequency_t>::SharedPtr getSetPwmFrequencyClient(void)
  {
    return setPwmFrequencyClient;
  }
  rclcpp::Client<HalPigpioSetGpioHigh_t>::SharedPtr getSetGpioHighClient(void)
  {
    return setGpioHighClient;
  }
  rclcpp::Client<HalPigpioSetGpioLow_t>::SharedPtr getSetGpioLowClient(void)
  {
    return setGpioLowClient;
  }
  rclcpp::Client<HalPigpioSendTriggerPulse_t>::SharedPtr getSendTriggerPulseClient(void)
  {
    return sendTriggerPulseClient;
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
