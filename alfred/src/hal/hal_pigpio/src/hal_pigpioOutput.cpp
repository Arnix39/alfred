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

namespace hal
{
namespace pigpio
{

using namespace std::placeholders;

void Pigpio::setPwmDutycycle(
  const std::shared_ptr<HalPigpioSetPwmDutycycle_t::Request> request,
  std::shared_ptr<HalPigpioSetPwmDutycycle_t::Response> response)
{
  if (set_PWM_dutycycle(pigpioHandle, request->gpio_id, request->dutycycle) == 0) {
    response->has_succeeded = true;
  } else {
    response->has_succeeded = false;
    RCLCPP_ERROR(get_logger(), "Failed to set PWM duty cycle for GPIO %u!", request->gpio_id);
  }
}

void Pigpio::setPwmFrequency(
  const std::shared_ptr<HalPigpioSetPwmFrequency_t::Request> request,
  std::shared_ptr<HalPigpioSetPwmFrequency_t::Response> response)
{
  if (set_PWM_frequency(pigpioHandle, request->gpio_id, request->frequency) >= 0) {
    response->has_succeeded = true;
    RCLCPP_INFO(
      get_logger(), "Set PWM frequency of %u for GPIO %u.", request->frequency, request->gpio_id);
  } else {
    response->has_succeeded = false;
    RCLCPP_ERROR(get_logger(), "Failed to set PWM frequency for GPIO %u!", request->gpio_id);
  }
}

void Pigpio::setGpioHigh(
  const std::shared_ptr<HalPigpioSetGpioHigh_t::Request> request,
  std::shared_ptr<HalPigpioSetGpioHigh_t::Response> response)
{
  if (gpio_write(pigpioHandle, request->gpio_id, PI_HIGH) == 0) {
    response->has_succeeded = true;
  } else {
    response->has_succeeded = false;
    RCLCPP_ERROR(get_logger(), "Failed to set GPIO %u to high level!", request->gpio_id);
  }
}

void Pigpio::setGpioLow(
  const std::shared_ptr<HalPigpioSetGpioLow_t::Request> request,
  std::shared_ptr<HalPigpioSetGpioLow_t::Response> response)
{
  if (gpio_write(pigpioHandle, request->gpio_id, PI_LOW) == 0) {
    response->has_succeeded = true;
  } else {
    response->has_succeeded = false;
    RCLCPP_ERROR(get_logger(), "Failed to set GPIO %u to low level!", request->gpio_id);
  }
}

void Pigpio::sendTriggerPulse(
  const std::shared_ptr<HalPigpioSendTriggerPulse_t::Request> request,
  std::shared_ptr<HalPigpioSendTriggerPulse_t::Response> response)
{
  if (gpio_trigger(pigpioHandle, request->gpio_id, request->pulse_length_in_us, PI_HIGH) == 0) {
    response->has_succeeded = true;
  } else {
    response->has_succeeded = false;
    RCLCPP_ERROR(get_logger(), "Failed to send trigger pulse for GPIO %u!", request->gpio_id);
  }
}

}  // namespace pigpio
}  // namespace hal
