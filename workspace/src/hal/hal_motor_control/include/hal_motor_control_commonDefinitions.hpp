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

#ifndef HAL_MOTOR_CONTROL_COMMONDEFINITIONS_HPP_
#define HAL_MOTOR_CONTROL_COMMONDEFINITIONS_HPP_

#include "common.hpp"

// Services and messages headers (generated)
#include "hal_motor_control_interfaces/msg/hal_motor_control.hpp"
#include "hal_pigpio_interfaces/msg/hal_pigpio_encoder_count.hpp"
#include "hal_pigpio_interfaces/srv/hal_pigpio_set_input_mode.hpp"
#include "hal_pigpio_interfaces/srv/hal_pigpio_set_output_mode.hpp"
#include "hal_pigpio_interfaces/srv/hal_pigpio_set_encoder_callback.hpp"
#include "hal_pigpio_interfaces/srv/hal_pigpio_set_pwm_frequency.hpp"
#include "hal_pigpio_interfaces/srv/hal_pigpio_set_pwm_dutycycle.hpp"
#include "hal_pigpio_interfaces/srv/hal_pigpio_set_motor_direction.hpp"

#define MOTOR_LEFT 0
#define MOTOR_LEFT_PWM_A_GPIO GPIO19
#define MOTOR_LEFT_PWM_B_GPIO GPIO16
#define MOTOR_LEFT_ENCODER_CH_A_GPIO GPIO17
#define MOTOR_LEFT_ENCODER_CH_B_GPIO GPIO18

#define MOTOR_RIGHT 1
#define MOTOR_RIGHT_PWM_A_GPIO GPIO26
#define MOTOR_RIGHT_PWM_B_GPIO GPIO20
#define MOTOR_RIGHT_ENCODER_CH_A_GPIO GPIO22
#define MOTOR_RIGHT_ENCODER_CH_B_GPIO GPIO23

#define MOTOR_PWM_FREQUENCY 1000

using HalMotorControlMsg_t = hal_motor_control_interfaces::msg::HalMotorControl;
using HalPigpioEncoderCountMsg_t = hal_pigpio_interfaces::msg::HalPigpioEncoderCount;

using HalPigpioSetInputMode_t = hal_pigpio_interfaces::srv::HalPigpioSetInputMode;
using HalPigpioSetOutputMode_t = hal_pigpio_interfaces::srv::HalPigpioSetOutputMode;
using HalPigpioSetEncoderCallback_t = hal_pigpio_interfaces::srv::HalPigpioSetEncoderCallback;
using HalPigpioSetPwmFrequency_t = hal_pigpio_interfaces::srv::HalPigpioSetPwmFrequency;
using HalPigpioSetPwmDutycycle_t = hal_pigpio_interfaces::srv::HalPigpioSetPwmDutycycle;
using HalPigpioSetMotorDirection_t = hal_pigpio_interfaces::srv::HalPigpioSetMotorDirection;

using SetInputModeFuture_t = rclcpp::Client<HalPigpioSetInputMode_t>::SharedFuture;
using SetOutputModeFuture_t = rclcpp::Client<HalPigpioSetOutputMode_t>::SharedFuture;
using SetEncoderCallbackFuture_t = rclcpp::Client<HalPigpioSetEncoderCallback_t>::SharedFuture;
using SetPwmFrequencyFuture_t = rclcpp::Client<HalPigpioSetPwmFrequency_t>::SharedFuture;
using SetPwmDutycycleFuture_t = rclcpp::Client<HalPigpioSetPwmDutycycle_t>::SharedFuture;
using SetMotorDirectionFuture_t = rclcpp::Client<HalPigpioSetMotorDirection_t>::SharedFuture;

#endif  // HAL_MOTOR_CONTROL_COMMONDEFINITIONS_HPP_
