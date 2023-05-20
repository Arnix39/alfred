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

#include "common.hpp"

// Services and messages headers (generated)

using ChangeStatePtr = rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr;

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
  void changeNodeToState(std::string node, std::uint8_t transition);

private:
  std::map<std::string, ChangeStatePtr> changeStateClients;
  std::vector<std::string> nodeList;
};

#endif  // HAL_LIFECYCLE_MANAGER_HPP_
