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

#include "hal_i2cRegistersServices_tests.hpp"

using namespace std::placeholders;
using namespace std::chrono_literals;

I2cRegistersServicesCheckerNode::I2cRegistersServicesCheckerNode()
: rclcpp::Node("hal_i2cRegistersServices_checker_node"),
  imuGetHandleSyncClient("getHandleSyncClientImu_node"),
  i2cReadByteDataSyncClient("readByteDataSyncClientImu_node"),
  i2cWriteByteDataSyncClient("writeByteDataSyncClientImu_node"),
  i2cWriteBlockDataSyncClient("writeBlockDataSyncClientImu_node")
{}

HalDummyNode::HalDummyNode()
: rclcpp::Node("hal_dummy_node"),
  piHandle(pigpio_start(NULL, NULL)),
  imuGetHandleService(this->create_service<HalImuGetHandle_t>(
      "hal_imuGetHandle",
      std::bind(&HalDummyNode::getHandle, this, _1, _2))),
  i2cReadByteDataService(this->create_service<HalPigpioI2cReadByteData_t>(
      "hal_pigpioI2cReadByteData",
      std::bind(&HalDummyNode::i2cReadByteData, this, _1, _2))),
  i2cWriteByteDataService(this->create_service<HalPigpioI2cWriteByteData_t>(
      "hal_pigpioI2cWriteByteData",
      std::bind(&HalDummyNode::i2cWriteByteData, this, _1, _2))),
  i2cWriteBlockDataService(this->create_service<HalPigpioI2cWriteBlockData_t>(
      "hal_pigpioI2cWriteBlockData",
      std::bind(&HalDummyNode::i2cWriteBlockData, this, _1, _2)))
{}

HalDummyNode::~HalDummyNode()
{
  pigpio_stop(piHandle);
}

void HalDummyNode::getHandle(
  const std::shared_ptr<HalImuGetHandle_t::Request> request,
  std::shared_ptr<HalImuGetHandle_t::Response> response)
{
  (void)request;

  response->handle = IMU_GOOD_HANDLE;
}

void HalDummyNode::i2cReadByteData(
  const std::shared_ptr<HalPigpioI2cReadByteData_t::Request> request,
  std::shared_ptr<HalPigpioI2cReadByteData_t::Response> response)
{
  int result = i2c_read_byte_data(piHandle, request->handle, request->device_register);
  if (result >= 0) {
    response->value = static_cast<uint8_t>(result);
    response->has_succeeded = true;
  } else {
    response->value = 0;
    response->has_succeeded = false;
  }
}

void HalDummyNode::i2cWriteByteData(
  const std::shared_ptr<HalPigpioI2cWriteByteData_t::Request> request,
  std::shared_ptr<HalPigpioI2cWriteByteData_t::Response> response)
{
  if (i2c_write_byte_data(
      piHandle, request->handle, request->device_register, request->value) == 0)
  {
    response->has_succeeded = true;
  } else {
    response->has_succeeded = false;
  }
}

void HalDummyNode::i2cWriteBlockData(
  const std::shared_ptr<HalPigpioI2cWriteBlockData_t::Request> request,
  std::shared_ptr<HalPigpioI2cWriteBlockData_t::Response> response)
{
  char dataBlock[I2C_BUFFER_MAX_BYTES];

  for (uint8_t byte = 0; byte < request->length; byte++) {
    dataBlock[byte] = static_cast<char>(request->data_block[byte]);
  }

  if (i2c_write_i2c_block_data(
      piHandle, request->handle, request->device_register, dataBlock, request->length) == 0)
  {
    response->has_succeeded = true;
  } else {
    response->has_succeeded = false;
  }
}

TEST_F(I2cRegistersServicesTest, getI2cHandleSuccess)
{
  auto future = std::async(
    std::launch::async, getI2cHandle,
    i2cRegistersServicesChecker->imuGetHandleSyncClient);

  auto status = executor.spin_until_future_complete(future);

  if (status == rclcpp::FutureReturnCode::SUCCESS) {
    ASSERT_EQ(future.get(), IMU_GOOD_HANDLE);
  } else {
    FAIL();
  }
}

TEST_F(I2cRegistersServicesTest, readByteFromRegisterSuccess)
{
  uint8_t valueToRead = 0x15;
  auto i2cHandle = i2c_open(
    halDummyNode->piHandle, I2C_GOOD_BUS_1, I2C_GOOD_ADDRESS, 0);
  i2c_write_byte_data(halDummyNode->piHandle, i2cHandle, I2C_GOOD_REGISTER, valueToRead);

  auto future = std::async(
    std::launch::async, readByteFromRegister,
    i2cRegistersServicesChecker->i2cReadByteDataSyncClient,
    i2cHandle,
    I2C_GOOD_REGISTER);

  auto status = executor.spin_until_future_complete(future);

  if (status == rclcpp::FutureReturnCode::SUCCESS) {
    ASSERT_EQ(future.get(), valueToRead);
  } else {
    FAIL();
  }

  i2c_close(halDummyNode->piHandle, i2cHandle);
}

TEST_F(I2cRegistersServicesTest, readByteFromRegisterFailure)
{
  auto i2cHandle = i2c_open(
    halDummyNode->piHandle, I2C_GOOD_BUS_1, I2C_GOOD_ADDRESS, 0);

  auto future = std::async(
    std::launch::async, readByteFromRegister,
    i2cRegistersServicesChecker->i2cReadByteDataSyncClient,
    i2cHandle,
    I2C_BAD_REGISTER);

  auto status = executor.spin_until_future_complete(future);

  if (status == rclcpp::FutureReturnCode::SUCCESS) {
    ASSERT_EQ(future.get(), -1);
  } else {
    FAIL();
  }

  i2c_close(halDummyNode->piHandle, i2cHandle);
}

TEST_F(I2cRegistersServicesTest, writeByteInRegisterSuccess)
{
  auto i2cHandle = i2c_open(
    halDummyNode->piHandle, I2C_GOOD_BUS_1, I2C_GOOD_ADDRESS, 0);

  auto future = std::async(
    std::launch::async, writeByteInRegister,
    i2cRegistersServicesChecker->i2cWriteByteDataSyncClient,
    i2cHandle,
    I2C_GOOD_REGISTER,
    0x15);

  auto status = executor.spin_until_future_complete(future);

  if (status == rclcpp::FutureReturnCode::SUCCESS) {
    ASSERT_EQ(future.get(), true);
  } else {
    FAIL();
  }

  i2c_close(halDummyNode->piHandle, i2cHandle);
}

TEST_F(I2cRegistersServicesTest, writeByteInRegisterFailure)
{
  auto i2cHandle = i2c_open(
    halDummyNode->piHandle, I2C_GOOD_BUS_1, I2C_GOOD_ADDRESS, 0);

  auto future = std::async(
    std::launch::async, writeByteInRegister,
    i2cRegistersServicesChecker->i2cWriteByteDataSyncClient,
    i2cHandle,
    I2C_BAD_REGISTER,
    0x15);

  auto status = executor.spin_until_future_complete(future);

  if (status == rclcpp::FutureReturnCode::SUCCESS) {
    ASSERT_EQ(future.get(), false);
  } else {
    FAIL();
  }

  i2c_close(halDummyNode->piHandle, i2cHandle);
}

TEST_F(I2cRegistersServicesTest, writeBitInRegisterSuccess)
{
  uint8_t bitToWrite = 4;
  uint8_t bitValue = 1;
  auto i2cHandle = i2c_open(
    halDummyNode->piHandle, I2C_GOOD_BUS_1, I2C_GOOD_ADDRESS, 0);
  i2c_write_byte_data(halDummyNode->piHandle, i2cHandle, I2C_GOOD_REGISTER, 0x0);

  auto future = std::async(
    std::launch::async, writeBitInRegister,
    i2cRegistersServicesChecker->i2cReadByteDataSyncClient,
    i2cRegistersServicesChecker->i2cWriteByteDataSyncClient,
    i2cHandle,
    I2C_GOOD_REGISTER,
    bitToWrite,
    bitValue);

  auto status = executor.spin_until_future_complete(future);

  if (status == rclcpp::FutureReturnCode::SUCCESS) {
    ASSERT_EQ(future.get(), true);
    ASSERT_EQ(
      i2c_read_byte_data(halDummyNode->piHandle, i2cHandle, I2C_GOOD_REGISTER),
      (bitValue << bitToWrite));
  } else {
    FAIL();
  }

  i2c_close(halDummyNode->piHandle, i2cHandle);
}

TEST_F(I2cRegistersServicesTest, writeBitInRegisterFailure)
{
  uint8_t bitToWrite = 4;
  uint8_t bitValue = 1;
  auto i2cHandle = i2c_open(
    halDummyNode->piHandle, I2C_GOOD_BUS_1, I2C_GOOD_ADDRESS, 0);

  auto future = std::async(
    std::launch::async, writeBitInRegister,
    i2cRegistersServicesChecker->i2cReadByteDataSyncClient,
    i2cRegistersServicesChecker->i2cWriteByteDataSyncClient,
    i2cHandle,
    I2C_BAD_REGISTER,
    bitToWrite,
    bitValue);

  auto status = executor.spin_until_future_complete(future);

  if (status == rclcpp::FutureReturnCode::SUCCESS) {
    ASSERT_EQ(future.get(), false);
  } else {
    FAIL();
  }

  i2c_close(halDummyNode->piHandle, i2cHandle);
}

TEST_F(I2cRegistersServicesTest, writeBlockInRegisterSuccess)
{
  char blockWritten[I2C_BUFFER_MAX_BYTES];
  auto i2cHandle = i2c_open(
    halDummyNode->piHandle, I2C_GOOD_BUS_1, I2C_GOOD_ADDRESS, 0);

  std::vector<uint8_t> data = {0x15, 0x14};

  auto future = std::async(
    std::launch::async, writeDataBlock,
    i2cRegistersServicesChecker->i2cWriteBlockDataSyncClient,
    i2cHandle,
    I2C_GOOD_REGISTER_2,
    data);

  auto status = executor.spin_until_future_complete(future);

  if (status == rclcpp::FutureReturnCode::SUCCESS) {
    ASSERT_EQ(future.get(), true);
  } else {
    FAIL();
  }

  i2c_close(halDummyNode->piHandle, i2cHandle);
}

TEST_F(I2cRegistersServicesTest, writeBlockInRegisterFailure)
{
  char blockWritten[I2C_BUFFER_MAX_BYTES];
  auto i2cHandle = i2c_open(
    halDummyNode->piHandle, I2C_GOOD_BUS_1, I2C_GOOD_ADDRESS, 0);

  std::vector<uint8_t> data = {0x15, 0x14};

  auto future = std::async(
    std::launch::async, writeDataBlock,
    i2cRegistersServicesChecker->i2cWriteBlockDataSyncClient,
    i2cHandle,
    I2C_GOOD_REGISTER,
    data);

  auto status = executor.spin_until_future_complete(future);

  if (status == rclcpp::FutureReturnCode::SUCCESS) {
    ASSERT_EQ(future.get(), false);
  } else {
    FAIL();
  }

  i2c_close(halDummyNode->piHandle, i2cHandle);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  auto result = RUN_ALL_TESTS();

  rclcpp::shutdown();
  return result;
}
