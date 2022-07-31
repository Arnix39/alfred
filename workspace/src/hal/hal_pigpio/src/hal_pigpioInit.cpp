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

#include "hal_pigpio.hpp"

void Pigpio::getMode(
  const std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioGetMode::Request> request,
  std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioGetMode::Response> response)
{
  response->mode = get_mode(pigpioHandle, request->gpio_id);
  if (response->mode >= 0) {
    response->has_succeeded = true;
  } else {
    response->has_succeeded = false;
    RCLCPP_ERROR(get_logger(), "Failed to retrieve mode for GPIO %u!", request->gpio_id);
  }
}

void Pigpio::setInputMode(
  const std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioSetInputMode::Request> request,
  std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioSetInputMode::Response> response)
{
  if (set_mode(pigpioHandle, request->gpio_id, PI_INPUT) == 0) {
    response->has_succeeded = true;
    RCLCPP_INFO(get_logger(), "GPIO %u configured as input.", request->gpio_id);
  } else {
    response->has_succeeded = false;
    RCLCPP_ERROR(get_logger(), "Failed to configure GPIO %u as input!", request->gpio_id);
  }
}

void Pigpio::setOutputMode(
  const std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioSetOutputMode::Request> request,
  std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioSetOutputMode::Response> response)
{
  if (set_mode(pigpioHandle, request->gpio_id, PI_OUTPUT) == 0) {
    response->has_succeeded = true;
    RCLCPP_INFO(get_logger(), "GPIO %u configured as output.", request->gpio_id);
  } else {
    response->has_succeeded = false;
    RCLCPP_ERROR(get_logger(), "Failed to configure GPIO %u as output!", request->gpio_id);
  }
}

void Pigpio::setPullUp(
  const std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioSetPullUp::Request> request,
  std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioSetPullUp::Response> response)
{
  if (set_pull_up_down(pigpioHandle, request->gpio_id, PI_PUD_UP) == 0) {
    response->has_succeeded = true;
    RCLCPP_INFO(get_logger(), "Sucessfully set pull-up resistor for GPIO %u.", request->gpio_id);
  } else {
    response->has_succeeded = true;
    RCLCPP_INFO(get_logger(), "Failed to set pull-up resistor for GPIO %u!", request->gpio_id);
  }
}

void Pigpio::setPullDown(
  const std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioSetPullDown::Request> request,
  std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioSetPullDown::Response> response)
{
  if (set_pull_up_down(pigpioHandle, request->gpio_id, PI_PUD_DOWN) == 0) {
    response->has_succeeded = true;
    RCLCPP_INFO(get_logger(), "Sucessfully set pull-down resistor for GPIO %u.", request->gpio_id);
  } else {
    response->has_succeeded = true;
    RCLCPP_INFO(get_logger(), "Failed to set pull-down resistor for GPIO %u!", request->gpio_id);
  }
}

void Pigpio::clearResistor(
  const std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioClearResistor::Request> request,
  std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioClearResistor::Response> response)
{
  if (set_pull_up_down(pigpioHandle, request->gpio_id, PI_PUD_OFF) == 0) {
    response->has_succeeded = true;
    RCLCPP_INFO(get_logger(), "Sucessfully clear resistor for GPIO %u.", request->gpio_id);
  } else {
    response->has_succeeded = true;
    RCLCPP_INFO(get_logger(), "Failed to clear resistor for GPIO %u!", request->gpio_id);
  }
}
