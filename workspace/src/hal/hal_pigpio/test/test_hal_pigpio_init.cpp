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
TEST_F(PigpioTest, SetInputModeSuccess)
{
  ASSERT_EQ(hal_pigpioGpioSet(GOOD_GPIO, pigioChecker->getSetInputModeClient(), &executor), true);
}

TEST_F(PigpioTest, SetInputModeFailure)
{
  ASSERT_EQ(hal_pigpioGpioSet(BAD_GPIO, pigioChecker->getSetInputModeClient(), &executor), false);
}

TEST_F(PigpioTest, SetOutputModeSuccess)
{
  ASSERT_EQ(hal_pigpioGpioSet(GOOD_GPIO, pigioChecker->getSetOutputModeClient(), &executor), true);
}

TEST_F(PigpioTest, SetOutputModeFailure)
{
  ASSERT_EQ(hal_pigpioGpioSet(BAD_GPIO, pigioChecker->getSetOutputModeClient(), &executor), false);
}

TEST_F(PigpioTest, GetModeSuccess)
{
  ASSERT_EQ(hal_pigpioGpioSet(GOOD_GPIO, pigioChecker->getGetModeClient(), &executor), true);
}

TEST_F(PigpioTest, GetModeFailure)
{
  ASSERT_EQ(hal_pigpioGpioSet(BAD_GPIO, pigioChecker->getGetModeClient(), &executor), false);
}

TEST_F(PigpioTest, SetPullUpSuccess)
{
  ASSERT_EQ(hal_pigpioGpioSet(GOOD_GPIO, pigioChecker->getSetPullUpClient(), &executor), true);
}

TEST_F(PigpioTest, SetPullUpFailure)
{
  ASSERT_EQ(hal_pigpioGpioSet(BAD_GPIO, pigioChecker->getSetPullUpClient(), &executor), false);
}

TEST_F(PigpioTest, SetPullDownSuccess)
{
  ASSERT_EQ(hal_pigpioGpioSet(GOOD_GPIO, pigioChecker->getSetPullDownClient(), &executor), true);
}

TEST_F(PigpioTest, SetPullDownFailure)
{
  ASSERT_EQ(hal_pigpioGpioSet(BAD_GPIO, pigioChecker->getSetPullDownClient(), &executor), false);
}

TEST_F(PigpioTest, ClearResistorSuccess)
{
  ASSERT_EQ(hal_pigpioGpioSet(GOOD_GPIO, pigioChecker->getClearResistorClient(), &executor), true);
}

TEST_F(PigpioTest, ClearResistorFailure)
{
  ASSERT_EQ(hal_pigpioGpioSet(BAD_GPIO, pigioChecker->getClearResistorClient(), &executor), false);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  auto result = RUN_ALL_TESTS();

  rclcpp::shutdown();
  return result;
}
