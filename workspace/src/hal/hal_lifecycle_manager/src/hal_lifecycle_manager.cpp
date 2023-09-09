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

using namespace std::chrono_literals;

namespace hal
{
namespace lifecycle_manager
{

LifecycleManager::LifecycleManager()
: rclcpp_lifecycle::LifecycleNode{"hal_lifecycle_manager"},
  changeStateClients{},
  transitionCallbacks{},
  changeStateSubscriptions{},
  nodesState{}
{
}

LifecycleCallbackReturn_t LifecycleManager::on_configure(
  const rclcpp_lifecycle::State & previous_state)
{
  createChangeStateClients(nodesList);
  createChangeStateCallbacks(nodesList);
  createChangeStateSubscriptions(nodesList);
  initializeNodesState(nodesList);
  RCLCPP_INFO(get_logger(), "Node configured!");

  return LifecycleCallbackReturn_t::SUCCESS;
}

LifecycleCallbackReturn_t LifecycleManager::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(get_logger(), "Node activated!");

  return LifecycleCallbackReturn_t::SUCCESS;
}

LifecycleCallbackReturn_t LifecycleManager::on_deactivate(
  const rclcpp_lifecycle::State & previous_state)
{
  for (auto node : nodesList) {
    std::thread thread([this, node]() {changeNodeToState(node, NodeState::Unconfigured);});
    thread.detach();
  }
  RCLCPP_INFO(get_logger(), "Node deactivated!");

  return LifecycleCallbackReturn_t::SUCCESS;
}

LifecycleCallbackReturn_t LifecycleManager::on_cleanup(
  const rclcpp_lifecycle::State & previous_state)
{
  changeStateClients.clear();
  transitionCallbacks.clear();
  changeStateSubscriptions.clear();
  RCLCPP_INFO(get_logger(), "Node unconfigured!");

  return LifecycleCallbackReturn_t::SUCCESS;
}

LifecycleCallbackReturn_t LifecycleManager::on_shutdown(
  const rclcpp_lifecycle::State & previous_state)
{
  for (auto node : nodesList) {
    std::thread thread([this, node]() {changeNodeToState(node, NodeState::Finalized);});
    thread.detach();
  }

  changeStateClients.clear();
  transitionCallbacks.clear();
  changeStateSubscriptions.clear();
  RCLCPP_INFO(get_logger(), "Node shutdown!");

  return LifecycleCallbackReturn_t::SUCCESS;
}

LifecycleCallbackReturn_t LifecycleManager::on_error(const rclcpp_lifecycle::State & previous_state)
{
  return LifecycleCallbackReturn_t::FAILURE;
}

void LifecycleManager::createChangeStateClients(std::vector<std::string> nodesList)
{
  for (auto node : nodesList) {
    changeStateClients.emplace(
      std::make_pair(
        node, this->create_client<lifecycle_msgs::srv::ChangeState>(node + "/change_state")));
  }
}

void LifecycleManager::createChangeStateCallbacks(std::vector<std::string> nodesList)
{
  for (auto node : nodesList) {
    ChangeStateCallback_t callback =
      [this, node](lifecycle_msgs::msg::TransitionEvent::ConstSharedPtr message) {
        std::lock_guard<std::mutex> guard(nodesStateMutex);
        auto state = message->goal_state.label;
        if (state == "unconfigured" || state == "inactive" ||
          state == "active" || state == "finalized")
        {
          nodesState[node] = getEnumFromString(state);
        }
      };

    transitionCallbacks.emplace(std::make_pair(node, callback));
  }
}

void LifecycleManager::createChangeStateSubscriptions(std::vector<std::string> nodesList)
{
  for (auto node : nodesList) {
    changeStateSubscriptions.emplace(
      std::make_pair(
        node,
        this->create_subscription<lifecycle_msgs::msg::TransitionEvent>(
          node + "/transition_event",
          10,
          transitionCallbacks.find(node)->second)));
  }
}

void LifecycleManager::initializeNodesState(std::vector<std::string> nodesList)
{
  for (auto node : nodesList) {
    nodesState.emplace(std::make_pair(node, NodeState::Unconfigured));
    expectedNodesState.emplace(std::make_pair(node, NodeState::Unconfigured));
  }
}

void LifecycleManager::changeNodeToState(std::string node, NodeState goalState)
{
  auto currentState = nodesState.find(node)->second;
  auto expectedState = expectedNodesState.find(node)->second;

  if (goalState != currentState && currentState != NodeState::Finalized) {
    auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    uint8_t transition;

    {
      std::lock_guard<std::mutex> guard(expectedNodesStateMutex);
      switch (currentState) {
        case NodeState::Unconfigured:
          switch (goalState) {
            case NodeState::Inactive:
            case NodeState::Active:
              expectedNodesState[node] = NodeState::Inactive;
              transition = lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE;
              break;
            case NodeState::Finalized:
              expectedNodesState[node] = NodeState::Finalized;
              transition = lifecycle_msgs::msg::Transition::TRANSITION_UNCONFIGURED_SHUTDOWN;
              break;
            default:
              RCLCPP_ERROR(get_logger(), "Node %s cannot reach requested state!", node.c_str());
              break;
          }
          break;
        case NodeState::Inactive:
          switch (goalState) {
            case NodeState::Unconfigured:
              expectedNodesState[node] = NodeState::Unconfigured;
              transition = lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP;
              break;
            case NodeState::Active:
              expectedNodesState[node] = NodeState::Active;
              transition = lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE;
              break;
            case NodeState::Finalized:
              expectedNodesState[node] = NodeState::Finalized;
              transition = lifecycle_msgs::msg::Transition::TRANSITION_INACTIVE_SHUTDOWN;
              break;
            default:
              RCLCPP_ERROR(get_logger(), "Node %s cannot reach requested state!", node.c_str());
              break;
          }
          break;
        case NodeState::Active:
          switch (goalState) {
            case NodeState::Unconfigured:
            case NodeState::Inactive:
              expectedNodesState[node] = NodeState::Inactive;
              transition = lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE;
              break;
            case NodeState::Finalized:
              expectedNodesState[node] = NodeState::Finalized;
              transition = lifecycle_msgs::msg::Transition::TRANSITION_ACTIVE_SHUTDOWN;
              break;
            default:
              RCLCPP_ERROR(get_logger(), "Node %s cannot reach requested state!", node.c_str());
              break;
          }
          break;
        case NodeState::Finalized:
          switch (goalState) {
            default:
              RCLCPP_ERROR(get_logger(), "Node %s cannot reach requested state!", node.c_str());
              break;
          }
          break;
        default:
          RCLCPP_ERROR(get_logger(), "Node %s is in an unknown state!", node.c_str());
          break;
      }
    }

    request->transition.id = transition;
    auto result = changeStateClients.find(node)->second->async_send_request(request).future.share();
    result.wait_for(5s);

    if (nodesState.find(node)->second != expectedNodesState.find(node)->second) {
      RCLCPP_ERROR(get_logger(), "Node %s couldn't reach expected state!", node.c_str());
    } else {
      changeNodeToState(node, goalState);
    }
  }
}

NodeState LifecycleManager::getEnumFromString(std::string state)
{
  return nodeStatesEnum.find(state)->second;
}

}  // namespace lifecycle_manager
}  // namespace hal
