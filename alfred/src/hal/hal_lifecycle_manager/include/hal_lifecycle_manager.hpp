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
#include <mutex>
#include <chrono>
#include <thread>

#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/msg/transition_event.hpp"

#include "common.hpp"

using ChangeStatePtr = rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr;
using TransitionMessage = lifecycle_msgs::msg::TransitionEvent;
using TransitionMessageSubscription = rclcpp::Subscription<TransitionMessage>::SharedPtr;
using ChangeStateCallback_t = std::function<void (TransitionMessage::ConstSharedPtr)>;

namespace hal
{
namespace lifecycle_manager
{

enum NodeState
{
  Unconfigured,
  Configuring,
  CleaningUp,
  Inactive,
  Activating,
  Deactivating,
  Active,
  ShuttingDown,
  Finalized,
  ErrorProcessing
};

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
  void createChangeStateCallbacks(std::vector<std::string> nodeList);
  void createChangeStateSubscriptions(std::vector<std::string> nodeList);
  void initializeNodesState(std::vector<std::string> nodeList);
  void changeNodeToState(std::string node, NodeState goalState);
  NodeState getEnumFromString(std::string state);

private:
  std::mutex nodesStateMutex;
  std::mutex expectedNodesStateMutex;
  const std::vector<std::string> nodesList{
    "hal_pigpio",
    "hal_proxsens",
    "hal_imuI2cInit",
    "hal_imuDmpWritingServer",
    "hal_imu",
    "hal_motor_control",
    "hal_pose_manager",
    "hal_camera"
  };
  const std::map<std::string, NodeState> nodeStatesEnum{
    {"unconfigured", NodeState::Unconfigured},
    {"configuring", NodeState::Configuring},
    {"cleaningUp", NodeState::CleaningUp},
    {"inactive", NodeState::Inactive},
    {"activating", NodeState::Activating},
    {"deactivating", NodeState::Deactivating},
    {"active", NodeState::Active},
    {"shuttingDown", NodeState::ShuttingDown},
    {"finalized", NodeState::Finalized},
    {"errorProcessing", NodeState::ErrorProcessing}
  };

  std::map<std::string, ChangeStatePtr> changeStateClients;
  std::map<std::string, ChangeStateCallback_t> transitionCallbacks;
  std::map<std::string, TransitionMessageSubscription> changeStateSubscriptions;
  std::map<std::string, NodeState> nodesState;
  std::map<std::string, NodeState> expectedNodesState;
};

}  // namespace lifecycle_manager
}  // namespace hal

#endif  // HAL_LIFECYCLE_MANAGER_HPP_
