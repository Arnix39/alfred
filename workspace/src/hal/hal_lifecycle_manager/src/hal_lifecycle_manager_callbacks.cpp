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

#include "hal_lifecycle_manager.hpp"

void LifecycleManager::hal_pigpioTransitionCallback(
  lifecycle_msgs::msg::TransitionEvent::ConstSharedPtr message)
{
  RCLCPP_INFO(
    get_logger(), "Node hal_pigpio transitions to state %s.",
    message->goal_state.label.c_str());
}

void LifecycleManager::hal_cameraTransitionCallback(
  lifecycle_msgs::msg::TransitionEvent::ConstSharedPtr message)
{
  RCLCPP_INFO(
    get_logger(), "Node hal_camera transitions to state %s.",
    message->goal_state.label.c_str());
}

void LifecycleManager::hal_proxsensTransitionCallback(
  lifecycle_msgs::msg::TransitionEvent::ConstSharedPtr message)
{
  RCLCPP_INFO(
    get_logger(), "Node hal_proxsens transitions to state %s.",
    message->goal_state.label.c_str());
}

void LifecycleManager::hal_imuI2cInitTransitionCallback(
  lifecycle_msgs::msg::TransitionEvent::ConstSharedPtr message)
{
  RCLCPP_INFO(
    get_logger(), "Node hal_imuI2cInit transitions to state %s.",
    message->goal_state.label.c_str());
}

void LifecycleManager::hal_imuDmpWritingServerTransitionCallback(
  lifecycle_msgs::msg::TransitionEvent::ConstSharedPtr message)
{
  RCLCPP_INFO(
    get_logger(), "Node hal_imuDmpWritingServer transitions to state %s.",
    message->goal_state.label.c_str());
}

void LifecycleManager::hal_imuTransitionCallback(
  lifecycle_msgs::msg::TransitionEvent::ConstSharedPtr message)
{
  RCLCPP_INFO(
    get_logger(), "Node hal_imu transitions to state %s.",
    message->goal_state.label.c_str());
}

void LifecycleManager::hal_motor_controlTransitionCallback(
  lifecycle_msgs::msg::TransitionEvent::ConstSharedPtr message)
{
  RCLCPP_INFO(
    get_logger(), "Node hal_motor_control transitions to state %s.",
    message->goal_state.label.c_str());
}

void LifecycleManager::hal_pose_managerTransitionCallback(
  lifecycle_msgs::msg::TransitionEvent::ConstSharedPtr message)
{
  RCLCPP_INFO(
    get_logger(), "Node hal_pose_manager transitions to state %s.",
    message->goal_state.label.c_str());
}
