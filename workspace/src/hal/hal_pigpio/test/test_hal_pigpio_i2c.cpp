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
TEST_F(PigpioTest, I2cOpenSuccess)
{
  ASSERT_GE(pigioChecker->i2cOpen(I2C_GOOD_BUS_1, I2C_GOOD_ADDRESS, &executor), 0);
}

TEST_F(PigpioTest, I2cOpenTwoBusesSuccess)
{
  ASSERT_GE(pigioChecker->i2cOpen(I2C_GOOD_BUS_1, I2C_GOOD_ADDRESS, &executor), 0);
  ASSERT_GE(pigioChecker->i2cOpen(I2C_GOOD_BUS_2, I2C_GOOD_ADDRESS, &executor), 0);
}

TEST_F(PigpioTest, I2cOpenBusFailure)
{
  ASSERT_EQ(pigioChecker->i2cOpen(I2C_BAD_BUS_2, I2C_GOOD_ADDRESS, &executor), PI_BAD_I2C_BUS);
}

TEST_F(PigpioTest, I2cOpenAddressFailure)
{
  ASSERT_EQ(pigioChecker->i2cOpen(I2C_GOOD_BUS_2, I2C_BAD_ADDRESS, &executor), PI_BAD_I2C_ADDR);
}

TEST_F(PigpioTest, I2cCloseSuccess)
{
  auto handle = pigioChecker->i2cOpen(I2C_GOOD_BUS_1, I2C_GOOD_ADDRESS, &executor);
  ASSERT_EQ(pigioChecker->i2cClose(handle, &executor), true);
}

TEST_F(PigpioTest, I2cCloseTwoBusesSuccess)
{
  auto handle = pigioChecker->i2cOpen(I2C_GOOD_BUS_1, I2C_GOOD_ADDRESS, &executor);
  auto handle2 = pigioChecker->i2cOpen(I2C_GOOD_BUS_2, I2C_GOOD_ADDRESS, &executor);
  ASSERT_EQ(pigioChecker->i2cClose(handle2, &executor), true);
}

TEST_F(PigpioTest, I2cCloseFailure)
{
  auto handle = pigioChecker->i2cOpen(I2C_GOOD_BUS_1, I2C_GOOD_ADDRESS, &executor);
  ASSERT_EQ(pigioChecker->i2cClose((handle + 1), &executor), false);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  auto result = RUN_ALL_TESTS();

  rclcpp::shutdown();
  return result;
}
