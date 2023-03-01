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

using namespace std::placeholders;

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
  pigpioEdgeChangeSub(this->create_subscription<HalPigpioEdgeChangeMsg_t>(
      "gpioEdgeChange",
      1000,
      std::bind(&PigioCheckerNode::edgeChangeCallback, this, std::placeholders::_1))),
  pigpioEncoderCountSub(this->create_subscription<HalPigpioEncoderCountMsg_t>(
      "hal_pigpioEncoderCount",
      1000,
      std::bind(&PigioCheckerNode::encoderCountCallback, this, std::placeholders::_1))),
  i2cOpenClient(this->create_client<HalPigpioI2cOpen_t>("hal_pigpioI2cOpen")),
  i2cCloseClient(this->create_client<HalPigpioI2cClose_t>("hal_pigpioI2cClose")),
  i2cReadByteDataClient(
    this->create_client<HalPigpioI2cReadByteData_t>("hal_pigpioI2cReadByteData")),
  i2cReadWordDataClient(
    this->create_client<HalPigpioI2cReadWordData_t>("hal_pigpioI2cReadWordData")),
  i2cReadBlockDataClient(
    this->create_client<HalPigpioI2cReadBlockData_t>("hal_pigpioI2cReadBlockData")),
  i2cWriteByteDataClient(
    this->create_client<HalPigpioI2cWriteByteData_t>("hal_pigpioI2cWriteByteData")),
  i2cWriteWordDataClient(
    this->create_client<HalPigpioI2cWriteWordData_t>("hal_pigpioI2cWriteWordData")),
  i2cWriteBlockDataClient(
    this->create_client<HalPigpioI2cWriteBlockData_t>("hal_pigpioI2cWriteBlockData")),
  i2cImuReadingClient(this->create_client<HalPigpioI2cImuReading_t>("hal_pigpioI2cImuReading")),
  anglesSubscriber(
    this->create_subscription<HalPigpioAnglesMsg_t>(
      "hal_pigpioAngles",
      1000,
      std::bind(&PigioCheckerNode::getAngles, this, _1))),
  edgeChangeMsg_gpioId(0),
  edgeChangeMsg_edgeChangeType(EdgeChangeType::undefined),
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
  EdgeChangeConfiguration edge_change_type,
  rclcpp::executors::SingleThreadedExecutor * executor)
{
  auto request = std::make_shared<HalPigpioSetCallback_t::Request>();

  request->gpio_id = gpio_id;
  request->edge_change_type = static_cast<uint8_t>(edge_change_type);

  auto future = setCallbackClient->async_send_request(request);

  executor->spin_until_future_complete(future);

  return future.get()->has_succeeded;
}

bool PigioCheckerNode::setEncoderCallback(
  uint16_t gpio_id,
  EdgeChangeConfiguration edge_change_type,
  uint8_t motor_id,
  EncoderChannel channel,
  rclcpp::executors::SingleThreadedExecutor * executor)
{
  auto request = std::make_shared<HalPigpioSetEncoderCallback_t::Request>();

  request->gpio_id = gpio_id;
  request->edge_change_type = static_cast<uint8_t>(edge_change_type);
  request->motor_id = motor_id;
  request->channel = static_cast<uint8_t>(channel);

  auto future = setEncoderCallbackClient->async_send_request(request);

  executor->spin_until_future_complete(future);

  return future.get()->has_succeeded;
}

void PigioCheckerNode::edgeChangeCallback(const HalPigpioEdgeChangeMsg_t & msg)
{
  edgeChangeMsg_gpioId = msg.gpio_id;
  edgeChangeMsg_edgeChangeType = static_cast<EdgeChangeType>(msg.edge_change_type);
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

int32_t PigioCheckerNode::i2cOpen(
  uint16_t bus,
  uint16_t address,
  rclcpp::executors::SingleThreadedExecutor * executor)
{
  auto request = std::make_shared<HalPigpioI2cOpen_t::Request>();

  request->bus = bus;
  request->address = address;

  auto future = i2cOpenClient->async_send_request(request);

  executor->spin_until_future_complete(future);

  return future.get()->handle;
}

bool PigioCheckerNode::i2cClose(
  int32_t handle,
  rclcpp::executors::SingleThreadedExecutor * executor)
{
  auto request = std::make_shared<HalPigpioI2cClose_t::Request>();

  request->handle = handle;

  auto future = i2cCloseClient->async_send_request(request);

  executor->spin_until_future_complete(future);

  return future.get()->has_succeeded;
}

bool PigioCheckerNode::i2cWriteByteData(
  int32_t i2cHandle,
  uint8_t deviceRegister,
  uint8_t value,
  rclcpp::executors::SingleThreadedExecutor * executor)
{
  auto request = std::make_shared<HalPigpioI2cWriteByteData_t::Request>();

  request->handle = i2cHandle;
  request->device_register = deviceRegister;
  request->value = value;

  auto future = i2cWriteByteDataClient->async_send_request(request);

  executor->spin_until_future_complete(future);

  return future.get()->has_succeeded;
}

bool PigioCheckerNode::i2cWriteWordData(
  int32_t i2cHandle,
  uint8_t deviceRegister,
  uint16_t value,
  rclcpp::executors::SingleThreadedExecutor * executor)
{
  auto request = std::make_shared<HalPigpioI2cWriteWordData_t::Request>();

  request->handle = i2cHandle;
  request->device_register = deviceRegister;
  request->value = value;

  auto future = i2cWriteWordDataClient->async_send_request(request);

  executor->spin_until_future_complete(future);

  return future.get()->has_succeeded;
}

bool PigioCheckerNode::i2cWriteBlockData(
  int32_t i2cHandle,
  uint8_t deviceRegister,
  std::vector<uint8_t> dataBlock,
  uint8_t length,
  rclcpp::executors::SingleThreadedExecutor * executor)
{
  auto request = std::make_shared<HalPigpioI2cWriteBlockData_t::Request>();

  request->handle = i2cHandle;
  request->device_register = deviceRegister;
  request->data_block = dataBlock;
  request->length = length;

  auto future = i2cWriteBlockDataClient->async_send_request(request);

  executor->spin_until_future_complete(future);

  return future.get()->has_succeeded;
}

uint8_t PigioCheckerNode::i2cReadByteData(
  int32_t i2cHandle,
  uint8_t deviceRegister,
  rclcpp::executors::SingleThreadedExecutor * executor)
{
  auto request = std::make_shared<HalPigpioI2cReadByteData_t::Request>();

  request->handle = i2cHandle;
  request->device_register = deviceRegister;

  auto future = i2cReadByteDataClient->async_send_request(request);

  executor->spin_until_future_complete(future);

  return future.get()->value;
}

uint16_t PigioCheckerNode::i2cReadWordData(
  int32_t i2cHandle,
  uint8_t deviceRegister,
  rclcpp::executors::SingleThreadedExecutor * executor)
{
  auto request = std::make_shared<HalPigpioI2cReadWordData_t::Request>();

  request->handle = i2cHandle;
  request->device_register = deviceRegister;

  auto future = i2cReadWordDataClient->async_send_request(request);

  executor->spin_until_future_complete(future);

  return future.get()->value;
}

std::vector<uint8_t> PigioCheckerNode::i2cReadBlockData(
  int32_t i2cHandle,
  uint8_t deviceRegister,
  uint8_t length,
  rclcpp::executors::SingleThreadedExecutor * executor)
{
  auto request = std::make_shared<HalPigpioI2cReadBlockData_t::Request>();

  request->handle = i2cHandle;
  request->device_register = deviceRegister;
  request->length = length;

  auto future = i2cReadBlockDataClient->async_send_request(request);

  executor->spin_until_future_complete(future);

  return future.get()->data_block;
}

void PigioCheckerNode::i2cStartImuReading(
  int32_t i2cHandle,
  rclcpp::executors::SingleThreadedExecutor * executor)
{
  auto request = std::make_shared<HalPigpioI2cImuReading_t::Request>();

  request->imu_handle = i2cHandle;
  request->is_imu_ready = true;

  auto future = i2cImuReadingClient->async_send_request(request);

  executor->spin_until_future_complete(future);
}

void PigioCheckerNode::i2cStopImuReading(
  int32_t i2cHandle,
  rclcpp::executors::SingleThreadedExecutor * executor)
{
  auto request = std::make_shared<HalPigpioI2cImuReading_t::Request>();

  request->imu_handle = i2cHandle;
  request->is_imu_ready = false;

  auto future = i2cImuReadingClient->async_send_request(request);

  executor->spin_until_future_complete(future);
}

void PigioCheckerNode::getAngles(const HalPigpioAnglesMsg_t & message)
{
  imuAnglePhi.set_value(message.phi);
  imuAngleTheta.set_value(message.theta);
  imuAnglePsi.set_value(message.psi);
}
