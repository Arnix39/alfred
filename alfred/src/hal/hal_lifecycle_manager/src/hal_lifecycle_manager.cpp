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
  for (auto node : nodesList) {
    std::thread thread([this, node]() {changeNodeToState(node.name, NodeState::Active);});
    thread.detach();
    if (node.waitRequiredAtStartup) {
      std::unique_lock<std::mutex> lock(nodesStateMutex);
      goalStateCondition.wait(
        lock, [this, node]() {
          return nodesState.find(node.name)->second == NodeState::Active;
        });
      lock.unlock();
    }
  }
  RCLCPP_INFO(get_logger(), "Node activated!");

  return LifecycleCallbackReturn_t::SUCCESS;
}

LifecycleCallbackReturn_t LifecycleManager::on_deactivate(
  const rclcpp_lifecycle::State & previous_state)
{
  for (auto node = nodesList.rbegin(); node <= nodesList.rend(); ++node) {
    std::thread thread([this, node]() {changeNodeToState(node->name, NodeState::Unconfigured);});
    thread.detach();
    if (node->waitRequiredAtShutdown) {
      std::unique_lock<std::mutex> lock(nodesStateMutex);
      goalStateCondition.wait(
        lock, [this, node]() {
          return nodesState.find(node->name)->second == NodeState::Unconfigured;
        });
      lock.unlock();
    }
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
  for (auto node = nodesList.rbegin(); node <= nodesList.rend(); ++node) {
    std::thread thread([this, node]() {changeNodeToState(node->name, NodeState::Finalized);});
    thread.detach();
    if (node->waitRequiredAtShutdown) {
      std::unique_lock<std::mutex> lock(nodesStateMutex);
      goalStateCondition.wait(
        lock, [this, node]() {
          return nodesState.find(node->name)->second == NodeState::Finalized;
        });
      lock.unlock();
    }

    changeStateSubscriptions[node->name].reset();
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

void LifecycleManager::createChangeStateClients(std::vector<hal::lifecycle_manager::node> nodesList)
{
  for (auto node : nodesList) {
    changeStateClients.emplace(
      std::make_pair(
        node.name,
        this->create_client<lifecycle_msgs::srv::ChangeState>(node.name + "/change_state")));
  }
}

void LifecycleManager::createChangeStateCallbacks(
  std::vector<hal::lifecycle_manager::node> nodesList)
{
  for (auto node : nodesList) {
    ChangeStateCallback_t callback =
      [this, node](lifecycle_msgs::msg::TransitionEvent::ConstSharedPtr message) {
        auto state = message->goal_state.label;
        if (state == "unconfigured" || state == "inactive" ||
          state == "active" || state == "finalized")
        {
          {
            std::lock_guard<std::mutex> guard(nodesStateMutex);
            nodesState[node.name] = getEnumFromString(state);
          }
          changeStateCondition.notify_all();
        }
      };

    transitionCallbacks.emplace(std::make_pair(node.name, callback));
  }
}

void LifecycleManager::createChangeStateSubscriptions(
  std::vector<hal::lifecycle_manager::node> nodesList)
{
  auto callback_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  rclcpp::SubscriptionOptions options;
  options.callback_group = callback_group;

  for (auto node : nodesList) {
    changeStateSubscriptions.emplace(
      std::make_pair(
        node.name,
        this->create_subscription<lifecycle_msgs::msg::TransitionEvent>(
          node.name + "/transition_event",
          10,
          transitionCallbacks.find(node.name)->second,
          options)));
  }
}

void LifecycleManager::initializeNodesState(std::vector<hal::lifecycle_manager::node> nodesList)
{
  for (auto node : nodesList) {
    nodesState.emplace(std::make_pair(node.name, NodeState::Unconfigured));
    expectedNodesState.emplace(std::make_pair(node.name, NodeState::Unconfigured));
  }
}

void LifecycleManager::changeNodeToState(std::string node, NodeState goalState)
{
  NodeState currentState = NodeState::Unknown;
  NodeState expectedState = NodeState::Unknown;
  ChangeStatePtr clientPtr = nullptr;

  {
    std::lock_guard<std::mutex> guard(nodesStateMutex);
    currentState = nodesState.find(node)->second;
  }
  {
    std::lock_guard<std::mutex> guard(expectedNodesStateMutex);
    expectedState = expectedNodesState.find(node)->second;
  }

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
              return;
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
              return;
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
              return;
          }
          break;
        default:
          RCLCPP_ERROR(get_logger(), "Node %s is in an unknown state!", node.c_str());
          return;
      }

      expectedState = expectedNodesState.find(node)->second;
    }

    request->transition.id = transition;
    {
      std::lock_guard<std::mutex> guard(changeStateClientsMutex);
      clientPtr = changeStateClients.find(node)->second;
    }
    auto resultFuture = clientPtr->async_send_request(request);

    std::unique_lock<std::mutex> lock(changeStateClientsMutex);
    changeStateCondition.wait(
      lock, [this, node, expectedState]() {
        return nodesState.find(node)->second == expectedState;
      });
    lock.unlock();

    if (expectedState != goalState) {
      changeNodeToState(node, goalState);
    } else {
      goalStateCondition.notify_all();
    }
  }
}

NodeState LifecycleManager::getEnumFromString(std::string state)
{
  std::lock_guard<std::mutex> guard(nodeStatesEnumMutex);
  return nodeStatesEnum.find(state)->second;
}

}  // namespace lifecycle_manager
}  // namespace hal
