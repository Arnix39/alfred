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

LifecycleManager::LifecycleManager()
: rclcpp_lifecycle::LifecycleNode{"hal_lifecycle_manager"},
  changeStateClients{}
{
  declare_parameter("node_list", rclcpp::PARAMETER_STRING_ARRAY);
  nodeList = get_parameter("node_list").as_string_array();
}

LifecycleCallbackReturn_t LifecycleManager::on_configure(
  const rclcpp_lifecycle::State & previous_state)
{
  createChangeStateClients(nodeList);
  RCLCPP_INFO(get_logger(), "hal_lifecycle_manager node configured!");

  return LifecycleCallbackReturn_t::SUCCESS;
}

LifecycleCallbackReturn_t LifecycleManager::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(get_logger(), "hal_lifecycle_manager node activated!");

  return LifecycleCallbackReturn_t::SUCCESS;
}

LifecycleCallbackReturn_t LifecycleManager::on_deactivate(
  const rclcpp_lifecycle::State & previous_state)
{
  changeStateClients.clear();
  RCLCPP_INFO(get_logger(), "hal_lifecycle_manager node deactivated!");

  return LifecycleCallbackReturn_t::SUCCESS;
}

LifecycleCallbackReturn_t LifecycleManager::on_cleanup(
  const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(get_logger(), "hal_lifecycle_manager node unconfigured!");

  return LifecycleCallbackReturn_t::SUCCESS;
}

LifecycleCallbackReturn_t LifecycleManager::on_shutdown(
  const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(get_logger(), "hal_lifecycle_manager node shutdown!");

  return LifecycleCallbackReturn_t::SUCCESS;
}

LifecycleCallbackReturn_t LifecycleManager::on_error(const rclcpp_lifecycle::State & previous_state)
{
  return LifecycleCallbackReturn_t::FAILURE;
}

void LifecycleManager::createChangeStateClients(std::vector<std::string> nodeList)
{
  for (auto node : nodeList) {
    changeStateClients.emplace(
      std::make_pair(
        node, this->create_client<lifecycle_msgs::srv::ChangeState>(node + "/change_state")));
  }
}

void LifecycleManager::changeNodeToState(std::string node, std::uint8_t transition)
{
  auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
  request->transition.id = transition;
  auto result = changeStateClients[node]->async_send_request(request);
}
