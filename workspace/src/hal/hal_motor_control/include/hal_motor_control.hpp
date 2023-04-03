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

#ifndef HAL_MOTOR_CONTROL_HPP_
#define HAL_MOTOR_CONTROL_HPP_

#include "hal_motor_control_commonDefinitions.hpp"
#include "hal_motor.hpp"

using setInputModeSyncClientNode_t = ServiceNodeSync<HalPigpioSetInputMode_t>;
using setOutputModeSyncClientNode_t = ServiceNodeSync<HalPigpioSetOutputMode_t>;
using setEncoderCallbackSyncClientNode_t = ServiceNodeSync<HalPigpioSetEncoderCallback_t>;
using setPwmFrequencySyncClientNode_t = ServiceNodeSync<HalPigpioSetPwmFrequency_t>;

class MotorControl : public rclcpp_lifecycle::LifecycleNode
{
private:
  Motor motorLeft;
  Motor motorRight;

  setInputModeSyncClientNode_t setInputModeSyncClient;
  setOutputModeSyncClientNode_t setOutputModeSyncClient;
  setEncoderCallbackSyncClientNode_t setEncoderCallbackSyncClient;
  setPwmFrequencySyncClientNode_t setPwmFrequencySyncClient;

  rclcpp::Client<HalPigpioSetPwmDutycycle_t>::SharedPtr gpioSetPwmDutycycleClient;

  rclcpp_lifecycle::LifecyclePublisher<PositionMsg_t>::SharedPtr positionPublisher;

  rclcpp::Subscription<HalPigpioEncoderCountMsg_t>::SharedPtr motorControlECSub;

  rclcpp::TimerBase::SharedPtr positionMessageTimer;

public:
  MotorControl();
  ~MotorControl() = default;

  LifecycleCallbackReturn_t on_configure(const rclcpp_lifecycle::State & previous_state);
  LifecycleCallbackReturn_t on_activate(const rclcpp_lifecycle::State & previous_state);
  LifecycleCallbackReturn_t on_deactivate(const rclcpp_lifecycle::State & previous_state);
  LifecycleCallbackReturn_t on_cleanup(const rclcpp_lifecycle::State & previous_state);
  LifecycleCallbackReturn_t on_shutdown(const rclcpp_lifecycle::State & previous_state);
  LifecycleCallbackReturn_t on_error(const rclcpp_lifecycle::State & previous_state);

  void activatePublisher(void);
  void configureMotor(void);
  void computeAndPublishPosition(void);
  void pigpioEncoderCountCallback(const HalPigpioEncoderCountMsg_t & msg);
  void setPwmLeft(uint16_t dutycycle, bool direction);
  void setPwmRight(uint16_t dutycycle, bool direction);
  double computeAbsolutePositionOnX(int32_t encoderCountLeft, int32_t encoderCountRight);
  double computeAbsolutePositionOnY(int32_t encoderCountLeft, int32_t encoderCountRight);
  double computeAbsolutePositionOnZ(int32_t encoderCountLeft, int32_t encoderCountRight);
};

#endif  // HAL_MOTOR_CONTROL_HPP_
