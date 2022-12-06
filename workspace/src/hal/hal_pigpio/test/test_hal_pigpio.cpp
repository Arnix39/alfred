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

#include "hal_pigpio_tests.hpp"

PigioCheckerNode::PigioCheckerNode()
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
      "hal_pigpioSetMotorDirection")),
  pigpioEdgeChangeSub(this->create_subscription<HalPigpioEdgeChangeMsg_t>(
      "gpioEdgeChange",
      1000,
      std::bind(&PigioCheckerNode::edgeChangeCallback, this, std::placeholders::_1))),
  pigpioEncoderCountSub(this->create_subscription<HalPigpioEncoderCountMsg_t>(
      "hal_pigpioEncoderCount",
      1000,
      std::bind(&PigioCheckerNode::encoderCountCallback, this, std::placeholders::_1))),
  edgeChangeMsg_gpioId(0),
  edgeChangeMsg_edgeChangeType(0),
  edgeChangeMsg_timeSinceBoot_us(0),
  motorsEC({})
{
}

void PigioCheckerNode::changePigpioNodeToState(std::uint8_t transition)
{
  auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
  request->transition.id = transition;
  auto result = changeStateClient->async_send_request(request);
}

bool PigioCheckerNode::setPwmDutycycle(
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

bool PigioCheckerNode::setPwmFrequency(
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

bool PigioCheckerNode::sendTriggerPulse(
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

bool PigioCheckerNode::readGpioAndCheckLevel(
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

bool PigioCheckerNode::setCallback(
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

bool PigioCheckerNode::setEncoderCallback(
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

bool PigioCheckerNode::setMotorDirection(
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

void PigioCheckerNode::edgeChangeCallback(const HalPigpioEdgeChangeMsg_t & msg)
{
  edgeChangeMsg_gpioId = msg.gpio_id;
  edgeChangeMsg_edgeChangeType = msg.edge_change_type;
  edgeChangeMsg_timeSinceBoot_us = msg.time_since_boot_us;
}

void PigioCheckerNode::encoderCountCallback(const HalPigpioEncoderCountMsg_t & msg)
{
  for (uint8_t index = 0; index < msg.motor_id.size(); ++index) {
    auto motorIndex = motorsEC.find(msg.motor_id[index]);
    if (motorIndex != motorsEC.end()) {
      motorIndex->second = msg.encoder_count[index];
    } else {
      motorsEC.insert({msg.motor_id[index], msg.encoder_count[index]});
    }
  }
}
