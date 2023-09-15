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

#include "hal_imuI2cInit_tests.hpp"

using namespace std::placeholders;

namespace hal
{
namespace imu
{
namespace i2cInit
{
namespace test
{

PigpioDummyNode::PigpioDummyNode()
: rclcpp::Node{"hal_pigpioDummy_node"},
  piHandle{pigpio_start(NULL, NULL)},
  i2cOpenService{this->create_service<HalPigpioI2cOpen_t>(
      "hal_pigpioI2cOpen", std::bind(&PigpioDummyNode::i2cOpen, this, _1, _2))},
  i2cCloseService{this->create_service<HalPigpioI2cClose_t>(
      "hal_pigpioI2cClose", std::bind(&PigpioDummyNode::i2cClose, this, _1, _2))}
{}

PigpioDummyNode::~PigpioDummyNode()
{
  pigpio_stop(piHandle);
}

void PigpioDummyNode::i2cOpen(
  const std::shared_ptr<HalPigpioI2cOpen_t::Request> request,
  std::shared_ptr<HalPigpioI2cOpen_t::Response> response)
{
  response->handle = i2c_open(piHandle, request->bus, request->address, 0);
  if (response->handle >= 0) {
    response->has_succeeded = true;
  } else {
    response->has_succeeded = false;
  }
}

void PigpioDummyNode::i2cClose(
  const std::shared_ptr<HalPigpioI2cClose_t::Request> request,
  std::shared_ptr<HalPigpioI2cClose_t::Response> response)
{
  if (i2c_close(piHandle, request->handle) == 0) {
    response->has_succeeded = true;
  } else {
    response->has_succeeded = false;
  }
}

ImuI2cInitCheckerNode::ImuI2cInitCheckerNode()
: rclcpp::Node("hal_imuI2cInit_checker_node"),
  changeStateClient(this->create_client<lifecycle_msgs::srv::ChangeState>(
      "hal_imuI2cInit_node/change_state")),
  hal_imuGetHandleClient(this->create_client<HalImuGetHandle_t>(
      "hal_imuGetHandle"))
{}

void ImuI2cInitCheckerNode::changeImuI2cInitNodeToState(std::uint8_t transition)
{
  auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
  request->transition.id = transition;
  auto result = changeStateClient->async_send_request(request);
}

TEST_F(ImuI2cInitTest, getHandle)
{
  int32_t handle;
  imuI2cInitChecker->changeImuI2cInitNodeToState(
    lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  executorImuI2cInit.spin_some();

  executorPigpio.spin_some();
  executorImuI2cInit.spin_some();

  auto request = std::make_shared<HalImuGetHandle_t::Request>();
  auto result = imuI2cInitChecker->hal_imuGetHandleClient->async_send_request(request);

  executorPigpio.spin_some();
  executorImuI2cInit.spin_until_future_complete(result);

  handle = result.get()->handle;

  imuI2cInitChecker->changeImuI2cInitNodeToState(
    lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
  executorImuI2cInit.spin_some();

  ASSERT_GE(handle, 0);
}

TEST_F(ImuI2cInitTest, initI2cCommunicationSuccess)
{
  int32_t handle;
  imuI2cInit->initI2cCommunication();

  executorPigpio.spin_some();
  executorImuI2cInit.spin_some();

  auto request = std::make_shared<HalImuGetHandle_t::Request>();
  auto result = imuI2cInitChecker->hal_imuGetHandleClient->async_send_request(request);

  executorPigpio.spin_some();
  executorImuI2cInit.spin_until_future_complete(result);

  handle = result.get()->handle;

  ASSERT_GE(handle, 0);
}

TEST_F(ImuI2cInitTest, initI2cCommunicationFailure)
{
  int32_t handle;
  pigpioDummy->piHandle = PI_BAD_HANDLE;

  imuI2cInit->initI2cCommunication();

  executorPigpio.spin_some();
  executorImuI2cInit.spin_some();

  auto request = std::make_shared<HalImuGetHandle_t::Request>();
  auto result = imuI2cInitChecker->hal_imuGetHandleClient->async_send_request(request);

  executorPigpio.spin_some();
  executorImuI2cInit.spin_until_future_complete(result);

  handle = result.get()->handle;

  ASSERT_EQ(handle, MPU6050_I2C_NO_HANDLE);
}

}  // namespace test
}  // namespace i2cInit
}  // namespace imu
}  // namespace hal

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  auto result = RUN_ALL_TESTS();

  rclcpp::shutdown();
  return result;
}
