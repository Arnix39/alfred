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

#include "hal_pigpio_tests.hpp"

using HalPigpioSetPwmDutycycle_t = hal_pigpio_interfaces::srv::HalPigpioSetPwmDutycycle;
using HalPigpioSetPwmFrequency_t = hal_pigpio_interfaces::srv::HalPigpioSetPwmFrequency;
using HalPigpioSetGpioHigh_t = hal_pigpio_interfaces::srv::HalPigpioSetGpioHigh;
using HalPigpioSetGpioLow_t = hal_pigpio_interfaces::srv::HalPigpioSetGpioLow;
using HalPigpioSendTriggerPulse_t = hal_pigpio_interfaces::srv::HalPigpioSendTriggerPulse;

class PigioOutputCheckerNode : public rclcpp::Node
{
private:
  rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr changeStateClient;
  rclcpp::Client<HalPigpioSetPwmDutycycle_t>::SharedPtr setPwmDutycycleClient;
  rclcpp::Client<HalPigpioSetPwmFrequency_t>::SharedPtr setPwmFrequencyClient;
  rclcpp::Client<HalPigpioSetGpioHigh_t>::SharedPtr setGpioHighClient;
  rclcpp::Client<HalPigpioSetGpioLow_t>::SharedPtr setGpioLowClient;
  rclcpp::Client<HalPigpioSendTriggerPulse_t>::SharedPtr sendTriggerPulseClient;

public:
  PigioOutputCheckerNode()
  : rclcpp::Node("hal_pigpio_checker_node"),
    changeStateClient(this->create_client<lifecycle_msgs::srv::ChangeState>(
        "hal_pigpio_node/change_state")),
    setPwmDutycycleClient(this->create_client<HalPigpioSetPwmDutycycle_t>(
        "hal_pigpioSetPwmDutycycle")),
    setPwmFrequencyClient(this->create_client<HalPigpioSetPwmFrequency_t>(
        "hal_pigpioSetPwmFrequency")),
    setGpioHighClient(this->create_client<HalPigpioSetGpioHigh_t>("hal_pigpioSetGpioHigh")),
    setGpioLowClient(this->create_client<HalPigpioSetGpioLow_t>("hal_pigpioSetGpioLow")),
    sendTriggerPulseClient(this->create_client<HalPigpioSendTriggerPulse_t>(
        "hal_pigpioSendTriggerPulse"))
  {
  }
  ~PigioOutputCheckerNode() = default;

  void changePigpioNodeToState(std::uint8_t transition)
  {
    auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    request->transition.id = transition;
    auto result = changeStateClient->async_send_request(request);
  }

  rclcpp::Client<HalPigpioSetPwmDutycycle_t>::SharedPtr getSetPwmDutycycleClient(void)
  {
    return setPwmDutycycleClient;
  }
  rclcpp::Client<HalPigpioSetPwmFrequency_t>::SharedPtr getSetPwmFrequencyClient(void)
  {
    return setPwmFrequencyClient;
  }
  rclcpp::Client<HalPigpioSetGpioHigh_t>::SharedPtr getSetGpioHighClient(void)
  {
    return setGpioHighClient;
  }
  rclcpp::Client<HalPigpioSetGpioLow_t>::SharedPtr getSetGpioLowClient(void)
  {
    return setGpioLowClient;
  }
  rclcpp::Client<HalPigpioSendTriggerPulse_t>::SharedPtr getSendTriggerPulseClient(void)
  {
    return sendTriggerPulseClient;
  }
};

using PigpioOutputTest = PigpioTest<PigioOutputCheckerNode>;

/* Test cases */
TEST_F(PigpioOutputTest, SetPwmDutycycleSuccess)
{
  ASSERT_EQ(hal_pigpioBoolTest(1, pigioChecker->getSetPwmDutycycleClient(), &executor), true);
}

TEST_F(PigpioOutputTest, SetPwmDutycycleFailure)
{
  ASSERT_EQ(hal_pigpioBoolTest(41, pigioChecker->getSetPwmDutycycleClient(), &executor), false);
}

TEST_F(PigpioOutputTest, SetPwmFrequencySuccess)
{
  ASSERT_EQ(hal_pigpioBoolTest(1, pigioChecker->getSetPwmFrequencyClient(), &executor), true);
}

TEST_F(PigpioOutputTest, SetPwmFrequencyFailure)
{
  ASSERT_EQ(hal_pigpioBoolTest(41, pigioChecker->getSetPwmFrequencyClient(), &executor), false);
}

TEST_F(PigpioOutputTest, SetGpioHighSuccess)
{
  ASSERT_EQ(hal_pigpioBoolTest(1, pigioChecker->getSetGpioHighClient(), &executor), true);
}

TEST_F(PigpioOutputTest, SetGpioHighFailure)
{
  ASSERT_EQ(hal_pigpioBoolTest(41, pigioChecker->getSetGpioHighClient(), &executor), false);
}

TEST_F(PigpioOutputTest, SetGpioLowSuccess)
{
  ASSERT_EQ(hal_pigpioBoolTest(1, pigioChecker->getSetGpioLowClient(), &executor), true);
}

TEST_F(PigpioOutputTest, SetGpioLowFailure)
{
  ASSERT_EQ(hal_pigpioBoolTest(41, pigioChecker->getSetGpioLowClient(), &executor), false);
}

TEST_F(PigpioOutputTest, SendTriggerPulseSuccess)
{
  ASSERT_EQ(hal_pigpioBoolTest(1, pigioChecker->getSendTriggerPulseClient(), &executor), true);
}

TEST_F(PigpioOutputTest, SendTriggerPulseFailure)
{
  ASSERT_EQ(hal_pigpioBoolTest(41, pigioChecker->getSendTriggerPulseClient(), &executor), false);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  auto result = RUN_ALL_TESTS();

  rclcpp::shutdown();
  return result;
}
