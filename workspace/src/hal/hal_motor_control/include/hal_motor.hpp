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

#ifndef HAL_MOTOR_HPP_
#define HAL_MOTOR_HPP_

#include "hal_motor_control_commonDefinitions.hpp"

using setInputModeSyncClientNode_t = ServiceNodeSync<HalPigpioSetInputMode_t>;
using setOutputModeSyncClientNode_t = ServiceNodeSync<HalPigpioSetOutputMode_t>;
using setEncoderCallbackSyncClientNode_t = ServiceNodeSync<HalPigpioSetEncoderCallback_t>;
using setPwmFrequencySyncClientNode_t = ServiceNodeSync<HalPigpioSetPwmFrequency_t>;

struct EncoderChannel
{
  uint8_t gpio;
};

struct Encoder
{
  EncoderChannel channelA;
  EncoderChannel channelB;
  uint32_t encoderCount;
};

struct Pwm
{
  uint8_t gpio;
  uint16_t dutycycle;
};

class Motor
{
private:
  Encoder encoder;
  Pwm pwmA;
  Pwm pwmB;
  uint8_t id;

public:
  Motor(
    uint8_t gpioPwmChannelA, uint8_t gpioPwmChannelB,
    uint8_t gpioEncoderChannelA, uint8_t gpioEncoderChannelB,
    uint8_t motorId);
  ~Motor() = default;

  uint32_t getEncoderCount(void);
  void setEncoderCount(uint32_t count);
  void setPwmDutyCycleAndDirection(
    rclcpp::Client<HalPigpioSetPwmDutycycle_t>::SharedPtr gpioSetPwmDutycycleClient,
    uint16_t dutycycle,
    rclcpp::Client<HalPigpioSetMotorDirection_t>::SharedPtr gpioSetMotorDirectionClient,
    bool isDirectionForward);
  void configureGpios(
    setOutputModeSyncClientNode_t gpioSetOutputModeClient,
    setInputModeSyncClientNode_t gpioSetInputModeClient,
    setEncoderCallbackSyncClientNode_t gpioSetEncoderCallbackClient,
    setPwmFrequencySyncClientNode_t gpioSetPwmFrequencyClient);
  uint8_t getId(void);
};

#endif  // HAL_MOTOR_HPP_
