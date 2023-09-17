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

namespace hal
{
namespace pigpio
{
namespace test
{

using namespace std::chrono_literals;

/* Test cases */
TEST_F(PigpioTest, ImuStartReading)
{
  auto handle = pigpioChecker->i2cOpen(I2C_GOOD_BUS_1, I2C_GOOD_ADDRESS, &executor);
  pigpioChecker->i2cWriteByteData(handle, MPU6050_USER_CONTROL_REGISTER, 0x0, &executor);
  pigpioChecker->i2cStartImuReading(handle, &executor);
  ASSERT_EQ(
    pigpioChecker->i2cReadByteData(handle, MPU6050_USER_CONTROL_REGISTER, &executor),
    MPU6050_FIFO_RESET);
}

TEST_F(PigpioTest, ImuResetFIFO)
{
  auto handle = pigpioChecker->i2cOpen(I2C_GOOD_BUS_1, I2C_GOOD_ADDRESS, &executor);
  pigpioChecker->i2cStartImuReading(handle, &executor);
  pigpioChecker->i2cWriteByteData(handle, MPU6050_USER_CONTROL_REGISTER, 0x0, &executor);
  pigpio->resetFifo();
  ASSERT_EQ(
    pigpioChecker->i2cReadByteData(handle, MPU6050_USER_CONTROL_REGISTER, &executor),
    MPU6050_FIFO_RESET);
}

TEST_F(PigpioTest, ImuReadFIFOCount)
{
  auto handle = pigpioChecker->i2cOpen(I2C_GOOD_BUS_1, I2C_GOOD_ADDRESS, &executor);
  pigpioChecker->i2cWriteByteData(handle, MPU6050_USER_CONTROL_REGISTER, 0x0, &executor);
  pigpioChecker->i2cStartImuReading(handle, &executor);
  pigpioChecker->i2cWriteByteData(handle, MPU6050_FIFO_COUNT_H_REGISTER, 0x15, &executor);
  pigpioChecker->i2cWriteByteData(handle, MPU6050_FIFO_COUNT_L_REGISTER, 0x14, &executor);
  ASSERT_EQ(pigpio->readFifoCount(), 0x1514);
}

TEST_F(PigpioTest, ImuFIFOOverflowed)
{
  auto handle = pigpioChecker->i2cOpen(I2C_GOOD_BUS_1, I2C_GOOD_ADDRESS, &executor);
  pigpioChecker->i2cWriteByteData(handle, MPU6050_USER_CONTROL_REGISTER, 0x0, &executor);
  pigpioChecker->i2cStartImuReading(handle, &executor);
  pigpioChecker->i2cWriteByteData(
    handle, MPU6050_INTERRUPT_STATUS_REGISTER, FIFO_OVERFLOWED, &executor);
  ASSERT_EQ(pigpio->isFifoOverflowed(), true);
}

TEST_F(PigpioTest, ImuFIFONotOverflowed)
{
  auto handle = pigpioChecker->i2cOpen(I2C_GOOD_BUS_1, I2C_GOOD_ADDRESS, &executor);
  pigpioChecker->i2cWriteByteData(handle, MPU6050_USER_CONTROL_REGISTER, 0x0, &executor);
  pigpioChecker->i2cStartImuReading(handle, &executor);
  pigpioChecker->i2cWriteByteData(
    handle, MPU6050_INTERRUPT_STATUS_REGISTER, FIFO_NOT_OVERFLOWED, &executor);
  ASSERT_EQ(pigpio->isFifoOverflowed(), false);
}

TEST_F(PigpioTest, ImuComputeAndPublishAnglesFIFOOverflowed)
{
  auto handle = pigpioChecker->i2cOpen(I2C_GOOD_BUS_1, I2C_GOOD_ADDRESS, &executor);
  pigpioChecker->i2cWriteByteData(handle, MPU6050_USER_CONTROL_REGISTER, 0x0, &executor);
  pigpioChecker->i2cStartImuReading(handle, &executor);
  pigpioChecker->i2cWriteByteData(
    handle, MPU6050_INTERRUPT_STATUS_REGISTER, FIFO_OVERFLOWED, &executor);
  pigpio->readImuDataAndPublishMessage();
  ASSERT_EQ(
    pigpioChecker->i2cReadByteData(handle, MPU6050_USER_CONTROL_REGISTER, &executor),
    MPU6050_FIFO_RESET);
}

}  // namespace test
}  // namespace pigpio
}  // namespace hal

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  auto result = RUN_ALL_TESTS();

  rclcpp::shutdown();
  return result;
}
