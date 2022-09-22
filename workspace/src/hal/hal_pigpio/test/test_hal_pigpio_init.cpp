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

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"

#include "hal_pigpio.hpp"

class PigioCheckerNode : public rclcpp::Node
{
public:
  PigioCheckerNode()
  : rclcpp::Node("hal_pigpio_checker_node"),
    changeStateClient(this->create_client<lifecycle_msgs::srv::ChangeState>(
        "hal_pigpio_node/change_state")),
    setInputModeClient(this->create_client<hal_pigpio_interfaces::srv::HalPigpioSetInputMode>(
        "hal_pigpioSetInputMode"))
  {
  }
  ~PigioCheckerNode() = default;
  void changePigpioNodeToState(std::uint8_t transition)
  {
    auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    request->transition.id = transition;
    auto result = changeStateClient->async_send_request(request);
  }
  rclcpp::Client<hal_pigpio_interfaces::srv::HalPigpioSetInputMode>::SharedPtr
  getSetInputModeClient(void)
  {
    return setInputModeClient;
  }

private:
  rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr changeStateClient;
  rclcpp::Client<hal_pigpio_interfaces::srv::HalPigpioSetInputMode>::SharedPtr setInputModeClient;
};

/* Test fixture */
class PigpioInitTest : public testing::Test
{
protected:
  std::shared_ptr<Pigpio> pigpioInit;
  std::shared_ptr<PigioCheckerNode> pigioChecker;
  rclcpp::executors::SingleThreadedExecutor executor;

  void SetUp()
  {
    pigioChecker = std::make_shared<PigioCheckerNode>();
    pigpioInit = std::make_shared<Pigpio>();

    executor.add_node(pigpioInit->get_node_base_interface());
    executor.add_node(pigioChecker);

    pigioChecker->changePigpioNodeToState(
      lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    executor.spin_some();
    pigioChecker->changePigpioNodeToState(
      lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
    executor.spin_some();
  }

  void TearDown()
  {
    executor.cancel();
    executor.remove_node(pigpioInit->get_node_base_interface());
    executor.remove_node(pigioChecker);
    pigpioInit.reset();
    pigioChecker.reset();
  }
};

/* Test cases */
TEST_F(PigpioInitTest, SetInputMode)
{
  auto setInputModeRequest =
    std::make_shared<hal_pigpio_interfaces::srv::HalPigpioSetInputMode::Request>();

  setInputModeRequest->gpio_id = 1;

  auto setInputModeFuture = pigioChecker->getSetInputModeClient()->async_send_request(
    setInputModeRequest);

  // executor.spin_some();

  ASSERT_EQ(setInputModeFuture.get()->has_succeeded, true);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  auto result = RUN_ALL_TESTS();

  rclcpp::shutdown();
  return result;
}
