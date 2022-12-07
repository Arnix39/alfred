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

TEST_F(PigpioTest, I2cOpenFailure)
{
  pigioChecker->i2cOpen(I2C_GOOD_BUS_1, I2C_GOOD_ADDRESS, &executor);
  ASSERT_EQ(pigioChecker->i2cOpen(I2C_GOOD_BUS_1, I2C_GOOD_ADDRESS, &executor), PI_I2C_OPEN_FAILED);
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

TEST_F(PigpioTest, I2cWriteByteSuccess)
{
  auto handle = pigioChecker->i2cOpen(I2C_GOOD_BUS_1, I2C_GOOD_ADDRESS, &executor);
  ASSERT_EQ(pigioChecker->i2cWriteByteData(handle, I2C_GOOD_REGISTER, 0x15, &executor), true);
}

TEST_F(PigpioTest, I2cWriteByteFailure)
{
  auto handle = pigioChecker->i2cOpen(I2C_GOOD_BUS_1, I2C_GOOD_ADDRESS, &executor);
  ASSERT_EQ(pigioChecker->i2cWriteByteData(handle, I2C_BAD_REGISTER, 0x15, &executor), false);
}

TEST_F(PigpioTest, I2cWriteWordSuccess)
{
  auto handle = pigioChecker->i2cOpen(I2C_GOOD_BUS_1, I2C_GOOD_ADDRESS, &executor);
  ASSERT_EQ(pigioChecker->i2cWriteWordData(handle, I2C_GOOD_REGISTER_2, 0x1515, &executor), true);
}

TEST_F(PigpioTest, I2cWriteWordFailure)
{
  auto handle = pigioChecker->i2cOpen(I2C_GOOD_BUS_1, I2C_GOOD_ADDRESS_2, &executor);
  ASSERT_EQ(pigioChecker->i2cWriteWordData(handle, I2C_BAD_REGISTER, 0x1515, &executor), false);
}

TEST_F(PigpioTest, I2cWriteBlockSuccess)
{
  std::vector<uint8_t> dataBlock = {0x15, 0x15, 0x15, 0x15};
  auto handle = pigioChecker->i2cOpen(I2C_GOOD_BUS_1, I2C_GOOD_ADDRESS_2, &executor);
  ASSERT_EQ(
    pigioChecker->i2cWriteBlockData(
      handle, I2C_GOOD_REGISTER_2, dataBlock, dataBlock.size(), &executor), true);
}

TEST_F(PigpioTest, I2cWriteBlockFailure)
{
  std::vector<uint8_t> dataBlock = {0x15, 0x15, 0x15, 0x15, 0x15};
  auto handle = pigioChecker->i2cOpen(I2C_GOOD_BUS_1, I2C_GOOD_ADDRESS_2, &executor);
  ASSERT_EQ(
    pigioChecker->i2cWriteBlockData(
      handle, I2C_GOOD_REGISTER_2, dataBlock, dataBlock.size(), &executor), false);
}

TEST_F(PigpioTest, I2cReadByteSuccess)
{
  auto handle = pigioChecker->i2cOpen(I2C_GOOD_BUS_1, I2C_GOOD_ADDRESS, &executor);
  pigioChecker->i2cWriteByteData(handle, I2C_GOOD_REGISTER, 0x15, &executor);
  ASSERT_EQ(pigioChecker->i2cReadByteData(handle, I2C_GOOD_REGISTER, &executor), 0x15);
}

TEST_F(PigpioTest, I2cReadByteFailure)
{
  auto handle = pigioChecker->i2cOpen(I2C_GOOD_BUS_1, I2C_GOOD_ADDRESS, &executor);
  pigioChecker->i2cWriteByteData(handle, I2C_GOOD_REGISTER, 0x15, &executor);
  ASSERT_EQ(pigioChecker->i2cReadByteData(handle, I2C_BAD_REGISTER, &executor), 0x0);
}

TEST_F(PigpioTest, I2cReadWordSuccess)
{
  auto handle = pigioChecker->i2cOpen(I2C_GOOD_BUS_1, I2C_GOOD_ADDRESS, &executor);
  pigioChecker->i2cWriteWordData(handle, I2C_GOOD_REGISTER_2, 0x1514, &executor);
  ASSERT_EQ(pigioChecker->i2cReadWordData(handle, I2C_GOOD_REGISTER_2, &executor), 0x1514);
}

TEST_F(PigpioTest, I2cReadWordFailure)
{
  auto handle = pigioChecker->i2cOpen(I2C_GOOD_BUS_1, I2C_GOOD_ADDRESS, &executor);
  pigioChecker->i2cWriteWordData(handle, I2C_GOOD_REGISTER_2, 0x1514, &executor);
  ASSERT_EQ(pigioChecker->i2cReadWordData(handle, I2C_BAD_REGISTER, &executor), 0x0);
}

TEST_F(PigpioTest, I2cReadBlockSuccess)
{
  std::vector<uint8_t> dataBlock = {0x15, 0x16, 0x17, 0x18};
  auto handle = pigioChecker->i2cOpen(I2C_GOOD_BUS_1, I2C_GOOD_ADDRESS_2, &executor);
  pigioChecker->i2cWriteBlockData(
    handle, I2C_GOOD_REGISTER_2, dataBlock, dataBlock.size(), &executor);
  std::vector<uint8_t> dataBlockRead =
    pigioChecker->i2cReadBlockData(handle, I2C_GOOD_REGISTER_2, dataBlock.size(), &executor);
  ASSERT_EQ(dataBlockRead.size(), dataBlock.size());
  for (int index = 0; index < dataBlock.size(); ++index) {
    ASSERT_EQ(dataBlockRead.at(index), dataBlock.at(index));
  }
}

TEST_F(PigpioTest, I2cReadBlockFailure)
{
  std::vector<uint8_t> dataBlock = {0x15, 0x16, 0x17, 0x18};
  auto handle = pigioChecker->i2cOpen(I2C_GOOD_BUS_1, I2C_GOOD_ADDRESS_2, &executor);
  pigioChecker->i2cWriteBlockData(
    handle, I2C_GOOD_REGISTER_2, dataBlock, dataBlock.size(), &executor);
  std::vector<uint8_t> dataBlockRead =
    pigioChecker->i2cReadBlockData(handle, I2C_BAD_REGISTER, dataBlock.size(), &executor);
  ASSERT_EQ(dataBlockRead.size(), 0);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  auto result = RUN_ALL_TESTS();

  rclcpp::shutdown();
  return result;
}
