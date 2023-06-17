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

#ifndef HAL_MOTOR_CONTROL_TESTS_HPP_
#define HAL_MOTOR_CONTROL_TESTS_HPP_

#include <memory>
#include <future>
#include <vector>

#include "hal_motor_control_tests_helpers.hpp"
#include "hal_motor_control.hpp"

#define BAD_MOTOR_ID (MOTOR_RIGHT + 10)

class MotorControlCheckerNode : public rclcpp::Node
{
public:
  MotorControlCheckerNode();
  ~MotorControlCheckerNode() = default;

  rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr changeStateClient;
  rclcpp::Subscription<HalMotorControlEncodersMsg_t>::SharedPtr encoderCountSubscriber;

  std::vector<int32_t> encoderCounts;

  void changeMotorControlNodeToState(std::uint8_t transition);
  void encoderCountCallback(const HalMotorControlEncodersMsg_t & msg);
};

/* Test fixture */
class MotorControlTest : public testing::Test
{
protected:
  std::shared_ptr<MotorControl> motorControl;
  std::shared_ptr<HalPigpioDummyNode> pigpioDummy;
  std::shared_ptr<MotorControlCheckerNode> motorControlChecker;
  rclcpp::executors::SingleThreadedExecutor executor;

  void SetUp()
  {
    motorControl = std::make_shared<MotorControl>();
    pigpioDummy = std::make_shared<HalPigpioDummyNode>();
    motorControlChecker = std::make_shared<MotorControlCheckerNode>();

    executor.add_node(motorControl->get_node_base_interface());
    executor.add_node(motorControlChecker);
    executor.add_node(pigpioDummy);

    executor.spin_some();

    motorControlChecker->changeMotorControlNodeToState(
      lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    executor.spin_some();
  }

  void TearDown()
  {
    executor.cancel();
    executor.remove_node(motorControl->get_node_base_interface());
    executor.remove_node(motorControlChecker);
    executor.remove_node(pigpioDummy);
    pigpioDummy.reset();
    motorControlChecker.reset();
    motorControl.reset();
  }
};

#endif  // HAL_MOTOR_CONTROL_TESTS_HPP_
