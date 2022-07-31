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

class MotorControl : public rclcpp_lifecycle::LifecycleNode
{
private:
  Motor motorLeft;
  Motor motorRight;

  rclcpp::Client<hal_pigpio_interfaces::srv::HalPigpioSetInputMode>::SharedPtr gpioSetInputClient;
  rclcpp::Client<hal_pigpio_interfaces::srv::HalPigpioSetOutputMode>::SharedPtr gpioSetOutputClient;
  rclcpp::Client<hal_pigpio_interfaces::srv::HalPigpioSetEncoderCallback>::SharedPtr
    gpioSetEncoderCallbackClient;
  rclcpp::Client<hal_pigpio_interfaces::srv::HalPigpioSetPwmFrequency>::SharedPtr
    gpioSetPwmFrequencyClient;
  rclcpp::Client<hal_pigpio_interfaces::srv::HalPigpioSetPwmDutycycle>::SharedPtr
    gpioSetPwmDutycycleClient;
  rclcpp::Client<hal_pigpio_interfaces::srv::HalPigpioSetMotorDirection>::SharedPtr
    gpioSetMotorDirectionClient;

  rclcpp_lifecycle::LifecyclePublisher<hal_motor_control_interfaces::msg::HalMotorControl>::
  SharedPtr motorControlPub;

  rclcpp::Subscription<hal_pigpio_interfaces::msg::HalPigpioEncoderCount>::SharedPtr
    motorControlECSub;

  rclcpp::TimerBase::SharedPtr encoderCountsTimer;

public:
  MotorControl();
  ~MotorControl() = default;

  LifecycleCallbackReturn_t on_configure(const rclcpp_lifecycle::State & previous_state);
  LifecycleCallbackReturn_t on_activate(const rclcpp_lifecycle::State & previous_state);
  LifecycleCallbackReturn_t on_deactivate(const rclcpp_lifecycle::State & previous_state);
  LifecycleCallbackReturn_t on_cleanup(const rclcpp_lifecycle::State & previous_state);
  LifecycleCallbackReturn_t on_shutdown(const rclcpp_lifecycle::State & previous_state);
  LifecycleCallbackReturn_t on_error(const rclcpp_lifecycle::State & previous_state);

  void configureMotor(void);
  void publishMessage(void);
  void pigpioEncoderCountCallback(const hal_pigpio_interfaces::msg::HalPigpioEncoderCount & msg);
  void setPwmLeft(uint16_t dutycycle, bool direction);
  void setPwmRight(uint16_t dutycycle, bool direction);
};

#endif  // HAL_MOTOR_CONTROL_HPP_
