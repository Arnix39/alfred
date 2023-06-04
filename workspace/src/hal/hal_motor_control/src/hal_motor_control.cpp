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

#include "hal_motor_control.hpp"

using namespace std::placeholders;
using namespace std::chrono_literals;

MotorControl::MotorControl()
: rclcpp_lifecycle::LifecycleNode{"hal_motorControl_node"},
  motorLeft{MOTOR_LEFT_PWM_A_GPIO, MOTOR_LEFT_PWM_B_GPIO,
    MOTOR_LEFT_ENCODER_CH_A_GPIO, MOTOR_LEFT_ENCODER_CH_B_GPIO,
    MOTOR_LEFT},
  motorRight{MOTOR_RIGHT_PWM_A_GPIO, MOTOR_RIGHT_PWM_B_GPIO,
    MOTOR_RIGHT_ENCODER_CH_A_GPIO, MOTOR_RIGHT_ENCODER_CH_B_GPIO,
    MOTOR_RIGHT},
  setInputModeSyncClient{"setInputModeSyncClientMotor_node"},
  setOutputModeSyncClient{"setOutputModeSyncClientMotor_node"},
  setEncoderCallbackSyncClient{"setEncoderCallbackSyncClientMotor_node"},
  setPwmFrequencySyncClient{"setPwmFrequencySyncClientMotor_node"}
{
}

LifecycleCallbackReturn_t MotorControl::on_configure(const rclcpp_lifecycle::State & previous_state)
{
  setInputModeSyncClient.init("hal_pigpioSetInputMode");
  setOutputModeSyncClient.init("hal_pigpioSetOutputMode");
  setEncoderCallbackSyncClient.init("hal_pigpioSetEncoderCallback");
  setPwmFrequencySyncClient.init("hal_pigpioSetPwmFrequency");

  gpioSetPwmDutycycleClient =
    this->create_client<HalPigpioSetPwmDutycycle_t>("hal_pigpioSetPwmDutycycle");

  motorControlPub = this->create_publisher<HalMotorControlMsg_t>(
    "motorsEncoderCountValue", 1000);

  motorControlECSub = this->create_subscription<HalPigpioEncoderCountMsg_t>(
    "hal_pigpioEncoderCount", 10, std::bind(&MotorControl::pigpioEncoderCountCallback, this, _1));

  encoderCountsTimer = create_wall_timer(10ms, std::bind(&MotorControl::publishMessage, this));

  RCLCPP_INFO(get_logger(), "Node configured!");

  return LifecycleCallbackReturn_t::SUCCESS;
}

LifecycleCallbackReturn_t MotorControl::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  activatePublisher();
  configureMotor();

  RCLCPP_INFO(get_logger(), "Node activated!");

  return LifecycleCallbackReturn_t::SUCCESS;
}

LifecycleCallbackReturn_t MotorControl::on_deactivate(
  const rclcpp_lifecycle::State & previous_state)
{
  motorControlPub->on_deactivate();

  RCLCPP_INFO(get_logger(), "Node deactivated!");

  return LifecycleCallbackReturn_t::SUCCESS;
}

LifecycleCallbackReturn_t MotorControl::on_cleanup(const rclcpp_lifecycle::State & previous_state)
{
  motorControlPub.reset();
  encoderCountsTimer.reset();

  RCLCPP_INFO(get_logger(), "Node unconfigured!");

  return LifecycleCallbackReturn_t::SUCCESS;
}

LifecycleCallbackReturn_t MotorControl::on_shutdown(const rclcpp_lifecycle::State & previous_state)
{
  motorControlPub.reset();
  encoderCountsTimer.reset();

  RCLCPP_INFO(get_logger(), "Node shutdown!");

  return LifecycleCallbackReturn_t::SUCCESS;
}

LifecycleCallbackReturn_t MotorControl::on_error(const rclcpp_lifecycle::State & previous_state)
{
  return LifecycleCallbackReturn_t::FAILURE;
}

void MotorControl::activatePublisher(void)
{
  motorControlPub->on_activate();
}

void MotorControl::configureMotor(void)
{
  motorLeft.configureGpios(
    setOutputModeSyncClient, setInputModeSyncClient, setEncoderCallbackSyncClient,
    setPwmFrequencySyncClient);
  motorRight.configureGpios(
    setOutputModeSyncClient, setInputModeSyncClient, setEncoderCallbackSyncClient,
    setPwmFrequencySyncClient);
}

void MotorControl::pigpioEncoderCountCallback(
  const HalPigpioEncoderCountMsg_t & msg)
{
  for (uint8_t index = 0; index < msg.motor_id.size(); ++index) {
    if (msg.motor_id.at(index) == MOTOR_LEFT) {
      motorLeft.setEncoderCount(msg.encoder_count[index]);
    } else if (msg.motor_id.at(index) == MOTOR_RIGHT) {
      motorRight.setEncoderCount(msg.encoder_count[index]);
    } else {
      RCLCPP_ERROR(get_logger(), "Encoder count message received for unknown motor!");
    }
  }
}

void MotorControl::publishMessage(void)
{
  auto encoderCounts = HalMotorControlMsg_t();
  auto header = HeaderMsg_t();

  header.stamp = rclcpp::Clock().now();
  header.frame_id = "Wheels";
  encoderCounts.header = header;

  encoderCounts.motor_left_encoder_count = -motorLeft.getEncoderCount();
  encoderCounts.motor_right_encoder_count = motorRight.getEncoderCount();

  motorControlPub->publish(encoderCounts);
}

void MotorControl::setPwmLeft(uint16_t dutycycle, bool direction)
{
  motorLeft.setPwmDutyCycleAndDirection(gpioSetPwmDutycycleClient, dutycycle, direction);
}

void MotorControl::setPwmRight(uint16_t dutycycle, bool direction)
{
  motorRight.setPwmDutyCycleAndDirection(gpioSetPwmDutycycleClient, dutycycle, direction);
}
