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

#ifndef HAL_PROXSENS_HPP_
#define HAL_PROXSENS_HPP_

#include "common.hpp"

// Services and messages headers (generated)
#include "hal_pigpio_interfaces/srv/hal_pigpio_set_input_mode.hpp"
#include "hal_pigpio_interfaces/srv/hal_pigpio_set_output_mode.hpp"
#include "hal_pigpio_interfaces/srv/hal_pigpio_set_gpio_high.hpp"
#include "hal_pigpio_interfaces/srv/hal_pigpio_send_trigger_pulse.hpp"
#include "hal_pigpio_interfaces/srv/hal_pigpio_set_callback.hpp"
#include "hal_pigpio_interfaces/msg/hal_pigpio_edge_change.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "std_msgs/msg/header.hpp"

#define PROXSENS_TRIGGER_GPIO GPIO5
#define PROXSENS_ECHO_GPIO GPIO6
#define PROXSENS_LEVEL_SHIFTER_OE_GPIO GPIO10
#define PROXSENS_TRIGGER_LENGTH_US 20
#define PROXSENS_MIN_RANGE 0.002
#define PROXSENS_MAX_RANGE 4.0
#define PROXSENS_FOV 15

namespace hal
{
namespace proxsens
{

namespace pigpio_srv = hal_pigpio_interfaces::srv;
namespace pigpio_msg = hal_pigpio_interfaces::msg;

using SetInputModeFuture_t = rclcpp::Client<pigpio_srv::HalPigpioSetInputMode>::SharedFuture;
using SetCallbackFuture_t = rclcpp::Client<pigpio_srv::HalPigpioSetCallback>::SharedFuture;
using SetOutputModeFuture_t = rclcpp::Client<pigpio_srv::HalPigpioSetOutputMode>::SharedFuture;
using SendTriggerPulseFuture_t =
  rclcpp::Client<pigpio_srv::HalPigpioSendTriggerPulse>::SharedFuture;
using SetGpioHighFuture_t = rclcpp::Client<pigpio_srv::HalPigpioSetGpioHigh>::SharedFuture;
using ProxsensMsg_t = sensor_msgs::msg::Range;
using HeaderMsg_t = std_msgs::msg::Header;

class Proxsens : public rclcpp_lifecycle::LifecycleNode
{
private:
  EdgeChangeType edgeChangeType;
  uint32_t timestamp;
  uint32_t echoCallbackId;
  uint16_t distanceInCm;

  rclcpp::Client<pigpio_srv::HalPigpioSetInputMode>::SharedPtr gpioSetInputClient;
  rclcpp::Client<pigpio_srv::HalPigpioSetOutputMode>::SharedPtr gpioSetOutputClient;
  rclcpp::Client<pigpio_srv::HalPigpioSetCallback>::SharedPtr gpioSetCallbackClient;
  rclcpp::Client<pigpio_srv::HalPigpioSendTriggerPulse>::SharedPtr gpioSendTriggerPulseClient;
  rclcpp::Client<pigpio_srv::HalPigpioSetGpioHigh>::SharedPtr gpioSetGpioHighClient;

  rclcpp_lifecycle::LifecyclePublisher<ProxsensMsg_t>::SharedPtr proxsensDistancePub;
  rclcpp::TimerBase::SharedPtr proxsensDistancePubTimer;

  rclcpp::Subscription<pigpio_msg::HalPigpioEdgeChange>::SharedPtr proxsensEdgeChangeSub;

public:
  Proxsens();
  ~Proxsens() = default;

  LifecycleCallbackReturn_t on_configure(const rclcpp_lifecycle::State & previous_state);
  LifecycleCallbackReturn_t on_activate(const rclcpp_lifecycle::State & previous_state);
  LifecycleCallbackReturn_t on_deactivate(const rclcpp_lifecycle::State & previous_state);
  LifecycleCallbackReturn_t on_cleanup(const rclcpp_lifecycle::State & previous_state);
  LifecycleCallbackReturn_t on_shutdown(const rclcpp_lifecycle::State & previous_state);
  LifecycleCallbackReturn_t on_error(const rclcpp_lifecycle::State & previous_state);

  void publishDistance(void);
  void configureGpios(void);
  void trigger(void);
  void enableOutputLevelShifter(void);
  void publishAndGetDistance(void);

  void edgeChangeCallback(const pigpio_msg::HalPigpioEdgeChange & msg);
};

}  // namespace proxsens
}  // namespace hal


#endif  // HAL_PROXSENS_HPP_
