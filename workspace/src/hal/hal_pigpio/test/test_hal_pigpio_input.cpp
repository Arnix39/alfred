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
TEST_F(PigpioTest, ReadGpioSuccess)
{
  hal_pigpioGpioSet(GOOD_GPIO, pigioChecker->getSetGpioHighClient(), &executor);
  ASSERT_EQ(pigioChecker->readGpioAndCheckLevel(GOOD_GPIO, PI_HIGH, &executor), true);
}

TEST_F(PigpioTest, ReadGpioFailure)
{
  ASSERT_EQ(pigioChecker->readGpioAndCheckLevel(BAD_GPIO, PI_LOW, &executor), false);
}

TEST_F(PigpioTest, SetCallbackSuccess)
{
  hal_pigpioGpioSet(GOOD_GPIO, pigioChecker->getSetInputModeClient(), &executor);
  ASSERT_EQ(pigioChecker->setCallback(GOOD_GPIO, AS_EITHER_EDGE, &executor), true);
}

TEST_F(PigpioTest, SetCallbackFailure)
{
  hal_pigpioGpioSet(GOOD_GPIO, pigioChecker->getSetOutputModeClient(), &executor);
  ASSERT_EQ(pigioChecker->setCallback(GOOD_GPIO, AS_EITHER_EDGE, &executor), false);
}

TEST_F(PigpioTest, SetEncoderCallbackSuccess)
{
  hal_pigpioGpioSet(GOOD_GPIO, pigioChecker->getSetInputModeClient(), &executor);
  ASSERT_EQ(
    pigioChecker->setEncoderCallback(GOOD_GPIO, AS_EITHER_EDGE, MOTOR_ID_1, &executor),
    true);
}

TEST_F(PigpioTest, SetEncoderCallbackFailure)
{
  hal_pigpioGpioSet(GOOD_GPIO, pigioChecker->getSetOutputModeClient(), &executor);
  ASSERT_EQ(
    pigioChecker->setEncoderCallback(GOOD_GPIO, AS_EITHER_EDGE, MOTOR_ID_1, &executor),
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
