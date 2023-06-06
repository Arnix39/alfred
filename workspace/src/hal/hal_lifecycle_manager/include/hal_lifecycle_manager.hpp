// Copyright (c) 2023 Arnix Robotix
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

#ifndef HAL_LIFECYCLE_MANAGER_HPP_
#define HAL_LIFECYCLE_MANAGER_HPP_

#include <map>
#include <string>
#include <vector>

#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/msg/transition_event.hpp"

#include "common.hpp"

// Services and messages headers (generated)

using ChangeStatePtr = rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr;
using TransitionMessage = lifecycle_msgs::msg::TransitionEvent;
using TransitionMessageSubscription = rclcpp::Subscription<TransitionMessage>::SharedPtr;
using ChangeStateCallback_t = std::function<void (TransitionMessage::ConstSharedPtr)>;

class LifecycleManager : public rclcpp_lifecycle::LifecycleNode
{
public:
  LifecycleManager();
  ~LifecycleManager() = default;

  LifecycleCallbackReturn_t on_configure(const rclcpp_lifecycle::State & previous_state);
  LifecycleCallbackReturn_t on_activate(const rclcpp_lifecycle::State & previous_state);
  LifecycleCallbackReturn_t on_deactivate(const rclcpp_lifecycle::State & previous_state);
  LifecycleCallbackReturn_t on_cleanup(const rclcpp_lifecycle::State & previous_state);
  LifecycleCallbackReturn_t on_shutdown(const rclcpp_lifecycle::State & previous_state);
  LifecycleCallbackReturn_t on_error(const rclcpp_lifecycle::State & previous_state);

  void createChangeStateClients(std::vector<std::string> nodeList);
  void createChangeStateSubscriptions(std::vector<std::string> nodeList);
  void changeNodeToState(std::string node, std::uint8_t transition);

  /* Nodes transitions callbacks */
  void hal_pigpioTransitionCallback(
    lifecycle_msgs::msg::TransitionEvent::ConstSharedPtr message);
  void hal_cameraTransitionCallback(
    lifecycle_msgs::msg::TransitionEvent::ConstSharedPtr message);
  void hal_proxsensTransitionCallback(
    lifecycle_msgs::msg::TransitionEvent::ConstSharedPtr message);
  void hal_imuI2cInitTransitionCallback(
    lifecycle_msgs::msg::TransitionEvent::ConstSharedPtr message);
  void hal_imuDmpWritingServerTransitionCallback(
    lifecycle_msgs::msg::TransitionEvent::ConstSharedPtr message);
  void hal_imuTransitionCallback(
    lifecycle_msgs::msg::TransitionEvent::ConstSharedPtr message);
  void hal_motor_controlTransitionCallback(
    lifecycle_msgs::msg::TransitionEvent::ConstSharedPtr message);
  void hal_pose_managerTransitionCallback(
    lifecycle_msgs::msg::TransitionEvent::ConstSharedPtr message);

private:
  std::map<std::string, ChangeStatePtr> changeStateClients;
  std::map<std::string, TransitionMessageSubscription> changeStateSubscriptions;

  const std::vector<std::string> nodeList{
    "hal_pigpio",
    "hal_camera",
    "hal_proxsens",
    "hal_imuI2cInit",
    "hal_imuDmpWritingServer",
    "hal_imu",
    "hal_motor_control",
    "hal_pose_manager"
  };

  const std::map<std::string, ChangeStateCallback_t> transitionCallbacks{
    {"hal_pigpio", std::bind(
        &LifecycleManager::hal_pigpioTransitionCallback,
        this,
        std::placeholders::_1)},
    {"hal_camera", std::bind(
        &LifecycleManager::hal_cameraTransitionCallback,
        this,
        std::placeholders::_1)},
    {"hal_proxsens", std::bind(
        &LifecycleManager::hal_proxsensTransitionCallback,
        this,
        std::placeholders::_1)},
    {"hal_imuI2cInit", std::bind(
        &LifecycleManager::hal_imuI2cInitTransitionCallback,
        this,
        std::placeholders::_1)},
    {"hal_imuDmpWritingServer", std::bind(
        &LifecycleManager::hal_imuDmpWritingServerTransitionCallback,
        this,
        std::placeholders::_1)},
    {"hal_imu", std::bind(
        &LifecycleManager::hal_imuTransitionCallback,
        this,
        std::placeholders::_1)},
    {"hal_motor_control", std::bind(
        &LifecycleManager::hal_motor_controlTransitionCallback,
        this,
        std::placeholders::_1)},
    {"hal_pose_manager", std::bind(
        &LifecycleManager::hal_pose_managerTransitionCallback,
        this,
        std::placeholders::_1)}
  };
};

#endif  // HAL_LIFECYCLE_MANAGER_HPP_
