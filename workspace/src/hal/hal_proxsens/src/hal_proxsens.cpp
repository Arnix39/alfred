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

#include "hal_proxsens.hpp"

using namespace std::placeholders;
using namespace std::chrono_literals;

namespace hal_proxsens
{

Proxsens::Proxsens()
: rclcpp_lifecycle::LifecycleNode("hal_proxsens_node"),
  edgeChangeType(NO_CHANGE),
  timestamp(0),
  echoCallbackId(0),
  distanceInCm(UINT16_MAX)
{
}

LifecycleCallbackReturn_t Proxsens::on_configure(const rclcpp_lifecycle::State & previous_state)
{
  gpioSetInputClient =
    this->create_client<pigpio_srv::HalPigpioSetInputMode>("hal_pigpioSetInputMode");
  gpioSetOutputClient =
    this->create_client<pigpio_srv::HalPigpioSetOutputMode>("hal_pigpioSetOutputMode");
  gpioSetCallbackClient =
    this->create_client<pigpio_srv::HalPigpioSetCallback>("hal_pigpioSetCallback");
  gpioSendTriggerPulseClient =
    this->create_client<pigpio_srv::HalPigpioSendTriggerPulse>("hal_pigpioSendTriggerPulse");
  gpioSetGpioHighClient =
    this->create_client<pigpio_srv::HalPigpioSetGpioHigh>("hal_pigpioSetGpioHigh");

  proxsensDistancePub =
    this->create_publisher<proxsens_msg::HalProxsens>("proxSensorValue", 1000);

  proxsensEdgeChangeSub =
    this->create_subscription<pigpio_msg::HalPigpioEdgeChange>(
    "gpioEdgeChange", 1000,
    std::bind(&Proxsens::edgeChangeCallback, this, _1));

  RCLCPP_INFO(get_logger(), "hal_proxsens node configured!");

  return LifecycleCallbackReturn_t::SUCCESS;
}

LifecycleCallbackReturn_t Proxsens::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  proxsensDistancePub->on_activate();

  configureGpios();
  enableOutputLevelShifter();

  RCLCPP_INFO(get_logger(), "hal_proxsens node activated!");

  return LifecycleCallbackReturn_t::SUCCESS;
}

LifecycleCallbackReturn_t Proxsens::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  proxsensDistancePub->on_deactivate();

  edgeChangeType = NO_CHANGE;
  timestamp = 0;
  echoCallbackId = 0;
  distanceInCm = UINT16_MAX;

  RCLCPP_INFO(get_logger(), "hal_proxsens node deactivated!");

  return LifecycleCallbackReturn_t::SUCCESS;
}

LifecycleCallbackReturn_t Proxsens::on_cleanup(const rclcpp_lifecycle::State & previous_state)
{
  proxsensDistancePub.reset();

  RCLCPP_INFO(get_logger(), "hal_proxsens node unconfigured!");

  return LifecycleCallbackReturn_t::SUCCESS;
}

LifecycleCallbackReturn_t Proxsens::on_shutdown(const rclcpp_lifecycle::State & previous_state)
{
  proxsensDistancePub.reset();

  RCLCPP_INFO(get_logger(), "hal_proxsens node shutdown!");

  return LifecycleCallbackReturn_t::SUCCESS;
}

LifecycleCallbackReturn_t Proxsens::on_error(const rclcpp_lifecycle::State & previous_state)
{
  return LifecycleCallbackReturn_t::FAILURE;
}

void Proxsens::edgeChangeCallback(const pigpio_msg::HalPigpioEdgeChange & msg)
{
  static uint8_t lastEdgeChangeType = NO_CHANGE;
  static uint32_t lastTimestamp = 0;

  uint32_t edgeLength = 0;

  if (msg.gpio_id == PROXSENS_ECHO_GPIO) {
    edgeChangeType = msg.edge_change_type;
    timestamp = msg.time_since_boot_us;

    if ((edgeChangeType == FALLING_EDGE) && (lastEdgeChangeType == RISING_EDGE)) {
      if (timestamp < lastTimestamp) {
        edgeLength = UINT32_MAX - lastTimestamp + timestamp;
      } else {
        edgeLength = timestamp - lastTimestamp;
      }
      distanceInCm = static_cast<uint16_t>(edgeLength / 58.0);

      lastEdgeChangeType = edgeChangeType;
      lastTimestamp = timestamp;
    } else if (edgeChangeType == RISING_EDGE) {
      lastEdgeChangeType = edgeChangeType;
      lastTimestamp = timestamp;
    } else {
      lastEdgeChangeType = FALLING_EDGE;
      lastTimestamp = timestamp;
    }
  }
}

void Proxsens::publishDistance(void)
{
  auto distance = proxsens_msg::HalProxsens();

  distance.distance_in_cm = distanceInCm;
  proxsensDistancePub->publish(distance);
}

void Proxsens::configureGpios(void)
{
  auto setInputModeRequest = std::make_shared<pigpio_srv::HalPigpioSetInputMode::Request>();
  auto setOutputModeForTriggerRequest =
    std::make_shared<pigpio_srv::HalPigpioSetOutputMode::Request>();
  auto setOutputModeForShifterRequest =
    std::make_shared<pigpio_srv::HalPigpioSetOutputMode::Request>();
  auto setCallbackRequest = std::make_shared<pigpio_srv::HalPigpioSetCallback::Request>();

  setInputModeRequest->gpio_id = PROXSENS_ECHO_GPIO;
  auto setInputModeCallback = [this](SetInputModeFuture_t future)
    {
      if (!future.get()->has_succeeded) {
        RCLCPP_ERROR(get_logger(), "Failed to call service setInputMode");
      }
    };
  auto setInputModeFuture =
    gpioSetInputClient->async_send_request(setInputModeRequest, setInputModeCallback);

  setCallbackRequest->gpio_id = PROXSENS_ECHO_GPIO;
  setCallbackRequest->edge_change_type = AS_EITHER_EDGE;
  auto setCallbackCallback = [this](SetCallbackFuture_t future)
    {
      if (future.get()->has_succeeded) {
        echoCallbackId = future.get()->callback_id;
      } else {
        RCLCPP_ERROR(get_logger(), "Failed to call service setCallback");
      }
    };
  auto setCallbackResult =
    gpioSetCallbackClient->async_send_request(setCallbackRequest, setCallbackCallback);

  setOutputModeForTriggerRequest->gpio_id = PROXSENS_TRIGGER_GPIO;
  auto setOutputModeForTriggerCallback = [this](SetOutputModeFuture_t future)
    {
      if (!future.get()->has_succeeded) {
        RCLCPP_ERROR(get_logger(), "Failed to call service setOutputMode for trigger");
      }
    };
  auto setOutputModeForTriggerResult =
    gpioSetOutputClient->async_send_request(
    setOutputModeForTriggerRequest,
    setOutputModeForTriggerCallback);

  setOutputModeForShifterRequest->gpio_id = PROXSENS_LEVEL_SHIFTER_OE_GPIO;
  auto setOutputModeForShifterCallback = [this](SetOutputModeFuture_t future)
    {
      if (!future.get()->has_succeeded) {
        RCLCPP_ERROR(get_logger(), "Failed to call service setOutputMode for shifter");
      }
    };
  auto setOutputModeForShifterResult =
    gpioSetOutputClient->async_send_request(
    setOutputModeForShifterRequest,
    setOutputModeForShifterCallback);
}

void Proxsens::trigger(void)
{
  auto sendTriggerPulseRequest =
    std::make_shared<pigpio_srv::HalPigpioSendTriggerPulse::Request>();

  sendTriggerPulseRequest->gpio_id = PROXSENS_TRIGGER_GPIO;
  sendTriggerPulseRequest->pulse_length_in_us = PROXSENS_TRIGGER_LENGTH_US;
  auto sendTriggerPulseCallback = [this](SendTriggerPulseFuture_t future)
    {
      if (!future.get()->has_succeeded) {
        RCLCPP_ERROR(get_logger(), "Failed to call service sendTriggerPulse");
      }
    };
  auto sendTriggerPulseResult =
    gpioSendTriggerPulseClient->async_send_request(
    sendTriggerPulseRequest,
    sendTriggerPulseCallback);
}

void Proxsens::enableOutputLevelShifter(void)
{
  auto setGpioHighRequest = std::make_shared<pigpio_srv::HalPigpioSetGpioHigh::Request>();

  setGpioHighRequest->gpio_id = PROXSENS_LEVEL_SHIFTER_OE_GPIO;
  auto setGpioHighCallback = [this](SetGpioHighFuture_t future)
    {
      if (!future.get()->has_succeeded) {
        RCLCPP_ERROR(get_logger(), "Failed to call service sendTriggerPulse");
      }
    };
  auto setGpioHighResult =
    gpioSetGpioHighClient->async_send_request(setGpioHighRequest, setGpioHighCallback);
}

void Proxsens::publishAndGetDistance(void)
{
  publishDistance();
  trigger();
}

}  // namespace hal_proxsens
