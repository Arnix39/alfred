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
using setInputModeFuture_t = rclcpp::Client<HalPigpioSetInputMode_t>::SharedFuture;
using HalPigpioSetOutputMode_t = hal_pigpio_interfaces::srv::HalPigpioSetOutputMode;
using setOutputModeFuture_t = rclcpp::Client<HalPigpioSetOutputMode_t>::SharedFuture;
using HalPigpioGetMode_t = hal_pigpio_interfaces::srv::HalPigpioGetMode;
using getModeFuture_t = rclcpp::Client<HalPigpioGetMode_t>::SharedFuture;
using HalPigpioSetPullUp_t = hal_pigpio_interfaces::srv::HalPigpioSetPullUp;
using setPullUpFuture_t = rclcpp::Client<HalPigpioSetPullUp_t>::SharedFuture;
using HalPigpioSetPullDown_t = hal_pigpio_interfaces::srv::HalPigpioSetPullDown;
using setPullDownFuture_t = rclcpp::Client<HalPigpioSetPullDown_t>::SharedFuture;
using HalPigpioClearResistor_t = hal_pigpio_interfaces::srv::HalPigpioClearResistor;
using clearResistorFuture_t = rclcpp::Client<HalPigpioClearResistor_t>::SharedFuture;

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
  auto setInputModeRequest = std::make_shared<HalPigpioSetInputMode_t::Request>();

  setInputModeRequest->gpio_id = 1;

  auto setInputModeFuture = pigioChecker->getSetInputModeClient()->async_send_request(
    setInputModeRequest);

  executor.spin_until_future_complete(setInputModeFuture);

  ASSERT_EQ(setInputModeFuture.get()->has_succeeded, true);
}

TEST_F(PigpioInitTest, SetInputModeFailure)
{
  auto setInputModeRequest = std::make_shared<HalPigpioSetInputMode_t::Request>();

  setInputModeRequest->gpio_id = 41;

  auto setInputModeFuture = pigioChecker->getSetInputModeClient()->async_send_request(
    setInputModeRequest);

  executor.spin_until_future_complete(setInputModeFuture);

  ASSERT_EQ(setInputModeFuture.get()->has_succeeded, false);
}

TEST_F(PigpioInitTest, SetOutputModeSuccess)
{
  auto setOutputModeRequest = std::make_shared<HalPigpioSetOutputMode_t::Request>();

  setOutputModeRequest->gpio_id = 1;

  auto setOutputModeFuture = pigioChecker->getSetOutputModeClient()->async_send_request(
    setOutputModeRequest);

  executor.spin_until_future_complete(setOutputModeFuture);

  ASSERT_EQ(setOutputModeFuture.get()->has_succeeded, true);
}

TEST_F(PigpioInitTest, SetOutputModeFailure)
{
  auto setOutputModeRequest = std::make_shared<HalPigpioSetOutputMode_t::Request>();

  setOutputModeRequest->gpio_id = 41;

  auto setOutputModeFuture = pigioChecker->getSetOutputModeClient()->async_send_request(
    setOutputModeRequest);

  executor.spin_until_future_complete(setOutputModeFuture);

  ASSERT_EQ(setOutputModeFuture.get()->has_succeeded, false);
}

TEST_F(PigpioInitTest, GetModeSuccess)
{
  auto getModeRequest = std::make_shared<HalPigpioGetMode_t::Request>();

  getModeRequest->gpio_id = 1;

  auto getModeFuture = pigioChecker->getGetModeClient()->async_send_request(
    getModeRequest);

  executor.spin_until_future_complete(getModeFuture);

  ASSERT_EQ(getModeFuture.get()->has_succeeded, true);
}

TEST_F(PigpioInitTest, GetModeFailure)
{
  auto getModeRequest = std::make_shared<HalPigpioGetMode_t::Request>();

  getModeRequest->gpio_id = 41;

  auto getModeFuture = pigioChecker->getGetModeClient()->async_send_request(
    getModeRequest);

  executor.spin_until_future_complete(getModeFuture);

  ASSERT_EQ(getModeFuture.get()->has_succeeded, false);
}

TEST_F(PigpioInitTest, SetPullUpSuccess)
{
  auto setPullUpRequest = std::make_shared<HalPigpioSetPullUp_t::Request>();

  setPullUpRequest->gpio_id = 1;

  auto setPullUpFuture = pigioChecker->getSetPullUpClient()->async_send_request(
    setPullUpRequest);

  executor.spin_until_future_complete(setPullUpFuture);

  ASSERT_EQ(setPullUpFuture.get()->has_succeeded, true);
}

TEST_F(PigpioInitTest, SetPullUpFailure)
{
  auto setPullUpRequest = std::make_shared<HalPigpioSetPullUp_t::Request>();

  setPullUpRequest->gpio_id = 41;

  auto setPullUpFuture = pigioChecker->getSetPullUpClient()->async_send_request(
    setPullUpRequest);

  executor.spin_until_future_complete(setPullUpFuture);

  ASSERT_EQ(setPullUpFuture.get()->has_succeeded, false);
}

TEST_F(PigpioInitTest, SetPullDownSuccess)
{
  auto setPullDownRequest = std::make_shared<HalPigpioSetPullDown_t::Request>();

  setPullDownRequest->gpio_id = 1;

  auto setPullDownFuture = pigioChecker->getSetPullDownClient()->async_send_request(
    setPullDownRequest);

  executor.spin_until_future_complete(setPullDownFuture);

  ASSERT_EQ(setPullDownFuture.get()->has_succeeded, true);
}

TEST_F(PigpioInitTest, SetPullDownFailure)
{
  auto setPullDownRequest = std::make_shared<HalPigpioSetPullDown_t::Request>();

  setPullDownRequest->gpio_id = 41;

  auto setPullDownFuture = pigioChecker->getSetPullDownClient()->async_send_request(
    setPullDownRequest);

  executor.spin_until_future_complete(setPullDownFuture);

  ASSERT_EQ(setPullDownFuture.get()->has_succeeded, false);
}

TEST_F(PigpioInitTest, ClearResistorSuccess)
{
  auto clearResistorRequest = std::make_shared<HalPigpioClearResistor_t::Request>();

  clearResistorRequest->gpio_id = 1;

  auto clearResistorFuture = pigioChecker->getClearResistorClient()->async_send_request(
    clearResistorRequest);

  executor.spin_until_future_complete(clearResistorFuture);

  ASSERT_EQ(clearResistorFuture.get()->has_succeeded, true);
}

TEST_F(PigpioInitTest, ClearResistorFailure)
{
  auto clearResistorRequest = std::make_shared<HalPigpioClearResistor_t::Request>();

  clearResistorRequest->gpio_id = 41;

  auto clearResistorFuture = pigioChecker->getClearResistorClient()->async_send_request(
    clearResistorRequest);

  executor.spin_until_future_complete(clearResistorFuture);

  ASSERT_EQ(clearResistorFuture.get()->has_succeeded, false);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  auto result = RUN_ALL_TESTS();

  rclcpp::shutdown();
  return result;
}
