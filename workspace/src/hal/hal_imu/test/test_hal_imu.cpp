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

#include "hal_imu_tests.hpp"

ImuCheckerNode::ImuCheckerNode()
: rclcpp::Node("hal_imu_checker_node"),
  changeStateClient(this->create_client<lifecycle_msgs::srv::ChangeState>(
      "hal_imu_node/change_state")),
  i2cReadByteDataDummy("DummyReadByteData"),
  i2cWriteByteDataDummy("DummyWriteByteData"),
  i2cWriteBlockDataDummy("DummyWriteBlockData"),
  imuGetHandleDummy("DummyGetHandle")
{
}

void ImuCheckerNode::changeImuNodeToState(std::uint8_t transition)
{
  auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
  request->transition.id = transition;
  auto result = changeStateClient->async_send_request(request);
}

void ImuCheckerNode::writeByte(uint8_t imuRegister, uint8_t value)
{
  writeByteInRegister(i2cWriteByteDataDummy, imuHandle, imuRegister, value);
}

int16_t ImuCheckerNode::readByte(uint8_t imuRegister)
{
  return readByteFromRegister(i2cReadByteDataDummy, imuHandle, imuRegister);
}

TEST_F(ImuTest, resetImu)
{
  imuChecker->writeByte(MPU6050_SIGNAL_PATH_RESET_REGISTER, 0x0);
  imuChecker->writeByte(MPU6050_POWER_MANAGEMENT_1_REGISTER, 0x1);

  imu->resetImu(imuChecker->imuHandle);

  ASSERT_EQ(imuChecker->readByte(MPU6050_SIGNAL_PATH_RESET_REGISTER), MPU6050_SIGNAL_PATH_RESET);
  ASSERT_EQ(imuChecker->readByte(MPU6050_POWER_MANAGEMENT_1_REGISTER), 0x0);
}

TEST_F(ImuTest, resetFifo)
{
  imuChecker->writeByte(MPU6050_USER_CONTROL_REGISTER, 0x0);

  imu->resetFifo(imuChecker->imuHandle);

  ASSERT_EQ(imuChecker->readByte(MPU6050_USER_CONTROL_REGISTER), MPU6050_FIFO_RESET);
}

TEST_F(ImuTest, setClockSource)
{
  imuChecker->writeByte(MPU6050_POWER_MANAGEMENT_1_REGISTER, 0x0);

  imu->setClockSource(imuChecker->imuHandle);

  ASSERT_EQ(imuChecker->readByte(MPU6050_POWER_MANAGEMENT_1_REGISTER), MPU6050_CLOCK_SOURCE_PLL_X);
}

TEST_F(ImuTest, setMpuRate)
{
  uint16_t rate = 100;
  uint8_t div = MPU6050_MAX_SAMPLE_RATE / rate - 1;

  imuChecker->writeByte(MPU6050_SAMPLE_RATE_REGISTER, 0x0);

  imu->setMpuRate(imuChecker->imuHandle, rate);

  ASSERT_EQ(imuChecker->readByte(MPU6050_SAMPLE_RATE_REGISTER), div);
}

TEST_F(ImuTest, setConfiguration)
{
  imuChecker->writeByte(MPU6050_CONFIGURATION_REGISTER, 0x0);

  imu->setConfiguration(imuChecker->imuHandle);

  ASSERT_EQ(imuChecker->readByte(MPU6050_CONFIGURATION_REGISTER), MPU6050_DLPF_BANDWITH_188);
}

TEST_F(ImuTest, setAccelerometerSensitivity)
{
  imuChecker->writeByte(MPU6050_ACCELEROMETER_CONFIGURATION_REGISTER, 0x1);

  imu->setAccelerometerSensitivity(imuChecker->imuHandle);

  ASSERT_EQ(
    imuChecker->readByte(MPU6050_ACCELEROMETER_CONFIGURATION_REGISTER),
    MPU6050_ACCELEROMETER_FULL_SENSITIVITY);
}

TEST_F(ImuTest, setGyroscopeSensitivity)
{
  imuChecker->writeByte(MPU6050_GYROSCOPE_CONFIGURATION_REGISTER, 0x0);

  imu->setGyroscopeSensitivity(imuChecker->imuHandle);

  ASSERT_EQ(
    imuChecker->readByte(MPU6050_GYROSCOPE_CONFIGURATION_REGISTER),
    MPU6050_GYROSCOPE_FULL_SENSITIVITY);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  auto result = RUN_ALL_TESTS();

  rclcpp::shutdown();
  return result;
}
