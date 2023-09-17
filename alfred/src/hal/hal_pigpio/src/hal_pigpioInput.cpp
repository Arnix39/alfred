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

void Pigpio::readGpio(
  const std::shared_ptr<HalPigpioReadGpio_t::Request> request,
  std::shared_ptr<HalPigpioReadGpio_t::Response> response)
{
  response->level = gpio_read(pigpioHandle, request->gpio_id);
  if (response->level != PI_BAD_GPIO) {
    response->has_succeeded = true;
  } else {
    response->has_succeeded = false;
    RCLCPP_ERROR(get_logger(), "Failed to read GPIO %u!", request->gpio_id);
  }
}

void Pigpio::gpioEdgeChangeCallback(
  int handle, unsigned gpioId, unsigned edgeChangeType,
  uint32_t timeSinceBoot_us)
{
  (void)handle;

  auto edgeChange = HalPigpioEdgeChangeMsg_t();

  edgeChange.gpio_id = gpioId;
  edgeChange.edge_change_type = edgeChangeType;
  edgeChange.time_since_boot_us = timeSinceBoot_us;

  gpioEdgeChangePub->publish(edgeChange);
}

// This is to pass the pointer of the callback function gpioEdgeChangeCallback to the pigpio
// library API
void Pigpio::c_gpioEdgeChangeCallback(
  int handle, unsigned gpioId, unsigned edgeChangeType,
  uint32_t timeSinceBoot_us, void * userData)
{
  Pigpio * object = reinterpret_cast<Pigpio *>(userData);
  object->gpioEdgeChangeCallback(handle, gpioId, edgeChangeType, timeSinceBoot_us);
}

void Pigpio::setCallback(
  const std::shared_ptr<HalPigpioSetCallback_t::Request> request,
  std::shared_ptr<HalPigpioSetCallback_t::Response> response)
{
  response->callback_id = callback_ex(
    pigpioHandle, request->gpio_id, request->edge_change_type,
    c_gpioEdgeChangeCallback, reinterpret_cast<void *>(this));
  if (response->callback_id >= 0) {
    response->has_succeeded = true;
    callbackList.push_back((uint)response->callback_id);
    RCLCPP_INFO(get_logger(), "Callback for GPIO %u configured.", request->gpio_id);
  } else {
    response->has_succeeded = false;
    RCLCPP_ERROR(get_logger(), "Failed to configure callback for GPIO %u!", request->gpio_id);
  }
}

void Pigpio::publishEncoderCount(void)
{
  auto encoderCount = HalPigpioEncoderCountMsg_t();

  if (motors.size() != 0) {
    for (Motor & motor : motors) {
      encoderCount.motor_id.push_back(motor.id);
      encoderCount.encoder_count.push_back(motor.encoderCount);
    }
    gpioEncoderCountPub->publish(encoderCount);
  }
}

MotorDirection Pigpio::computeDirection(
  const EncoderChannel & previousChannel, const EncoderChannel & channel,
  const EdgeChangeType & previousEdgeChange, const EdgeChangeType & edgeChange)
{
  if ((previousChannel != channel) && (previousChannel != EncoderChannel::undefined)) {
    switch (channel) {
      case EncoderChannel::channelA:
        if (previousEdgeChange == edgeChange) {
          return MotorDirection::backward;
        } else {
          return MotorDirection::forward;
        }
      case EncoderChannel::channelB:
        if (previousEdgeChange == edgeChange) {
          return MotorDirection::forward;
        } else {
          return MotorDirection::backward;
        }
      default:
        return MotorDirection::undefined;
    }
  } else {
    return MotorDirection::undefined;
  }
}

void Pigpio::gpioEncoderEdgeChangeCallback(
  int handle, unsigned gpioId, unsigned edgeChangeType,
  uint32_t timeSinceBoot_us)
{
  (void)handle;
  (void)timeSinceBoot_us;

  EncoderChannel channel;
  MotorDirection direction;

  for (Motor & motor : motors) {
    auto iterator = find(motor.gpios.begin(), motor.gpios.end(), gpioId);
    if (iterator != motor.gpios.end()) {
      channel = static_cast<EncoderChannel>(iterator - motor.gpios.begin());

      direction = computeDirection(
        motor.previousChannel, channel,
        motor.previousEdgeChangeType, static_cast<EdgeChangeType>(edgeChangeType));

      if (direction == MotorDirection::forward) {
        ++motor.encoderCount;
      } else if (direction == MotorDirection::backward) {
        --motor.encoderCount;
      } else {
        // direction is undefined, not changing encoder count
      }
      motor.previousChannel = channel;
      motor.previousEdgeChangeType = static_cast<EdgeChangeType>(edgeChangeType);
    } else {
      // Nothing to do
    }
  }
}

// This is to pass the pointer of the callback function gpioEncoderEdgeChangeCallback to the
// pigpio library API
void Pigpio::c_gpioEncoderEdgeChangeCallback(
  int handle, unsigned gpioId, unsigned edgeChangeType,
  uint32_t timeSinceBoot_us, void * userData)
{
  Pigpio * object = reinterpret_cast<Pigpio *>(userData);
  object->gpioEncoderEdgeChangeCallback(handle, gpioId, edgeChangeType, timeSinceBoot_us);
}

void Pigpio::setEncoderCallback(
  const std::shared_ptr<HalPigpioSetEncoderCallback_t::Request> request,
  std::shared_ptr<HalPigpioSetEncoderCallback_t::Response> response)
{
  response->callback_id = callback_ex(
    pigpioHandle, request->gpio_id, request->edge_change_type,
    c_gpioEncoderEdgeChangeCallback,
    reinterpret_cast<void *>(this));

  if (response->callback_id >= 0) {
    auto motorIndex = find_if(
      motors.begin(), motors.end(), [request](Motor motor) {
        return motor.id == request->motor_id;
      });

    if (motorIndex != motors.end()) {
      motors.at(motorIndex - motors.begin()).gpios.at(request->channel) = request->gpio_id;
    } else {
      if (static_cast<EncoderChannel>(request->channel) == EncoderChannel::channelA) {
        motors.push_back(
          {request->motor_id,
            {request->gpio_id, 0},
            EncoderChannel::undefined,
            EdgeChangeType::undefined,
            0});
      } else {
        motors.push_back(
          {request->motor_id,
            {0, request->gpio_id},
            EncoderChannel::undefined,
            EdgeChangeType::undefined,
            0});
      }
    }

    response->has_succeeded = true;
    callbackList.push_back((uint)response->callback_id);
    RCLCPP_INFO(get_logger(), "Encoder callback for GPIO %u configured.", request->gpio_id);
  } else {
    response->has_succeeded = false;
    RCLCPP_ERROR(
      get_logger(), "Failed to configure encoder callback for GPIO %u!",
      request->gpio_id);
  }
}

}  // namespace pigpio
}  // namespace hal
