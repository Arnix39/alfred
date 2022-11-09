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

using Transition_t = lifecycle_msgs::msg::Transition;
using HalPigpioSetInputMode_t = hal_pigpio_interfaces::srv::HalPigpioSetInputMode;
using HalPigpioSetOutputMode_t = hal_pigpio_interfaces::srv::HalPigpioSetOutputMode;
using HalPigpioGetMode_t = hal_pigpio_interfaces::srv::HalPigpioGetMode;
using HalPigpioSetPullUp_t = hal_pigpio_interfaces::srv::HalPigpioSetPullUp;
using HalPigpioSetPullDown_t = hal_pigpio_interfaces::srv::HalPigpioSetPullDown;
using HalPigpioClearResistor_t = hal_pigpio_interfaces::srv::HalPigpioClearResistor;

template<typename T>
bool hal_pigpioInitTest(
  uint8_t gpio_id,
  std::shared_ptr<rclcpp::Client<T>> serviceClient,
  rclcpp::executors::SingleThreadedExecutor * executor)
{
  auto request = std::make_shared<typename T::Request>();

  request->gpio_id = gpio_id;

  auto future = serviceClient->async_send_request(request);

  executor->spin_until_future_complete(future);

  return future.get()->has_succeeded;
}

class PigioCheckerNode : public rclcpp::Node
{
private:
  rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr changeStateClient;
  rclcpp::Client<HalPigpioSetInputMode_t>::SharedPtr setInputModeClient;
  rclcpp::Client<HalPigpioSetOutputMode_t>::SharedPtr setOutputModeClient;
  rclcpp::Client<HalPigpioGetMode_t>::SharedPtr getModeClient;
  rclcpp::Client<HalPigpioSetPullUp_t>::SharedPtr setPullUpClient;
  rclcpp::Client<HalPigpioSetPullDown_t>::SharedPtr setPullDownClient;
  rclcpp::Client<HalPigpioClearResistor_t>::SharedPtr clearResistorClient;

public:
  PigioCheckerNode()
  : rclcpp::Node("hal_pigpio_checker_node"),
    changeStateClient(this->create_client<lifecycle_msgs::srv::ChangeState>(
        "hal_pigpio_node/change_state")),
    setInputModeClient(this->create_client<HalPigpioSetInputMode_t>("hal_pigpioSetInputMode")),
    setOutputModeClient(this->create_client<HalPigpioSetOutputMode_t>("hal_pigpioSetOutputMode")),
    getModeClient(this->create_client<HalPigpioGetMode_t>("hal_pigpioGetMode")),
    setPullUpClient(this->create_client<HalPigpioSetPullUp_t>("hal_pigpioSetPullUp")),
    setPullDownClient(this->create_client<HalPigpioSetPullDown_t>("hal_pigpioSetPullDown")),
    clearResistorClient(this->create_client<HalPigpioClearResistor_t>("hal_pigpioClearResistor"))
  {
  }
  ~PigioCheckerNode() = default;

  void changePigpioNodeToState(std::uint8_t transition)
  {
    auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    request->transition.id = transition;
    auto result = changeStateClient->async_send_request(request);
  }

  rclcpp::Client<HalPigpioSetInputMode_t>::SharedPtr getSetInputModeClient(void)
  {
    return setInputModeClient;
  }
  rclcpp::Client<HalPigpioSetOutputMode_t>::SharedPtr getSetOutputModeClient(void)
  {
    return setOutputModeClient;
  }
  rclcpp::Client<HalPigpioGetMode_t>::SharedPtr getGetModeClient(void)
  {
    return getModeClient;
  }
  rclcpp::Client<HalPigpioSetPullUp_t>::SharedPtr getSetPullUpClient(void)
  {
    return setPullUpClient;
  }
  rclcpp::Client<HalPigpioSetPullDown_t>::SharedPtr getSetPullDownClient(void)
  {
    return setPullDownClient;
  }
  rclcpp::Client<HalPigpioClearResistor_t>::SharedPtr getClearResistorClient(void)
  {
    return clearResistorClient;
  }
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

    pigioChecker->changePigpioNodeToState(Transition_t::TRANSITION_CONFIGURE);
    executor.spin_some();
    pigioChecker->changePigpioNodeToState(Transition_t::TRANSITION_ACTIVATE);
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
TEST_F(PigpioInitTest, SetInputModeSuccess)
{
  ASSERT_EQ(hal_pigpioInitTest(1, pigioChecker->getSetInputModeClient(), &executor), true);
}

TEST_F(PigpioInitTest, SetInputModeFailure)
{
  ASSERT_EQ(hal_pigpioInitTest(41, pigioChecker->getSetInputModeClient(), &executor), false);
}

TEST_F(PigpioInitTest, SetOutputModeSuccess)
{
  ASSERT_EQ(hal_pigpioInitTest(1, pigioChecker->getSetOutputModeClient(), &executor), true);
}

TEST_F(PigpioInitTest, SetOutputModeFailure)
{
  ASSERT_EQ(hal_pigpioInitTest(41, pigioChecker->getSetOutputModeClient(), &executor), false);
}

TEST_F(PigpioInitTest, GetModeSuccess)
{
  ASSERT_EQ(hal_pigpioInitTest(1, pigioChecker->getGetModeClient(), &executor), true);
}

TEST_F(PigpioInitTest, GetModeFailure)
{
  ASSERT_EQ(hal_pigpioInitTest(41, pigioChecker->getGetModeClient(), &executor), false);
}

TEST_F(PigpioInitTest, SetPullUpSuccess)
{
  ASSERT_EQ(hal_pigpioInitTest(1, pigioChecker->getSetPullUpClient(), &executor), true);
}

TEST_F(PigpioInitTest, SetPullUpFailure)
{
  ASSERT_EQ(hal_pigpioInitTest(41, pigioChecker->getSetPullUpClient(), &executor), false);
}

TEST_F(PigpioInitTest, SetPullDownSuccess)
{
  ASSERT_EQ(hal_pigpioInitTest(1, pigioChecker->getSetPullDownClient(), &executor), true);
}

TEST_F(PigpioInitTest, SetPullDownFailure)
{
  ASSERT_EQ(hal_pigpioInitTest(41, pigioChecker->getSetPullDownClient(), &executor), false);
}

TEST_F(PigpioInitTest, ClearResistorSuccess)
{
  ASSERT_EQ(hal_pigpioInitTest(1, pigioChecker->getClearResistorClient(), &executor), true);
}

TEST_F(PigpioInitTest, ClearResistorFailure)
{
  ASSERT_EQ(hal_pigpioInitTest(41, pigioChecker->getClearResistorClient(), &executor), false);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  auto result = RUN_ALL_TESTS();

  rclcpp::shutdown();
  return result;
}
