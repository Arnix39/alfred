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

/* Test cases */
TEST_F(PigpioTest, SetPwmDutycycleSuccess)
{
  hal_pigpioGpioSet(GOOD_GPIO, pigioChecker->setOutputModeClient, &executor);
  ASSERT_EQ(
    pigioChecker->setPwmDutycycle(GOOD_GPIO, 50, &executor),
    true);
}

TEST_F(PigpioTest, SetPwmDutycycleFailure)
{
  hal_pigpioGpioSet(GOOD_GPIO, pigioChecker->setInputModeClient, &executor);
  ASSERT_EQ(
    pigioChecker->setPwmDutycycle(GOOD_GPIO, 50, &executor),
    false);
}

TEST_F(PigpioTest, SetPwmFrequencySuccess)
{
  hal_pigpioGpioSet(GOOD_GPIO, pigioChecker->setOutputModeClient, &executor);
  ASSERT_EQ(
    pigioChecker->setPwmFrequency(GOOD_GPIO, 1000, &executor),
    true);
}

TEST_F(PigpioTest, SetPwmFrequencyFailure)
{
  hal_pigpioGpioSet(GOOD_GPIO, pigioChecker->setInputModeClient, &executor);
  ASSERT_EQ(
    pigioChecker->setPwmFrequency(GOOD_GPIO, 1000, &executor),
    false);
}

TEST_F(PigpioTest, SetGpioHighSuccess)
{
  hal_pigpioGpioSet(GOOD_GPIO, pigioChecker->setOutputModeClient, &executor);
  ASSERT_EQ(hal_pigpioGpioSet(GOOD_GPIO, pigioChecker->setGpioHighClient, &executor), true);
}

TEST_F(PigpioTest, SetGpioHighFailure)
{
  hal_pigpioGpioSet(GOOD_GPIO, pigioChecker->setInputModeClient, &executor);
  ASSERT_EQ(hal_pigpioGpioSet(GOOD_GPIO, pigioChecker->setGpioHighClient, &executor), false);
}

TEST_F(PigpioTest, SetGpioLowSuccess)
{
  hal_pigpioGpioSet(GOOD_GPIO, pigioChecker->setOutputModeClient, &executor);
  ASSERT_EQ(hal_pigpioGpioSet(GOOD_GPIO, pigioChecker->setGpioLowClient, &executor), true);
}

TEST_F(PigpioTest, SetGpioLowFailure)
{
  hal_pigpioGpioSet(GOOD_GPIO, pigioChecker->setInputModeClient, &executor);
  ASSERT_EQ(hal_pigpioGpioSet(GOOD_GPIO, pigioChecker->setGpioLowClient, &executor), false);
}

TEST_F(PigpioTest, SendTriggerPulseSuccess)
{
  hal_pigpioGpioSet(GOOD_GPIO, pigioChecker->setOutputModeClient, &executor);
  ASSERT_EQ(
    pigioChecker->sendTriggerPulse(GOOD_GPIO, 100, &executor),
    true);
}

TEST_F(PigpioTest, SendTriggerPulseFailure)
{
  hal_pigpioGpioSet(GOOD_GPIO, pigioChecker->setInputModeClient, &executor);
  ASSERT_EQ(
    pigioChecker->sendTriggerPulse(GOOD_GPIO, 100, &executor),
    false);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  auto result = RUN_ALL_TESTS();

  rclcpp::shutdown();
  return result;
}
