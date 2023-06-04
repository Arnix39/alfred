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

#include "hal_motor_control_tests.hpp"

using namespace std::placeholders;

MotorControlCheckerNode::MotorControlCheckerNode()
: rclcpp::Node{"hal_motor_control_checker_node"},
  changeStateClient{this->create_client<lifecycle_msgs::srv::ChangeState>(
      "hal_motorControl_node/change_state")},
  encoderCountSubscriber{this->create_subscription<HalMotorControlMsg_t>(
      "motorsEncoderCountValue", 10,
      std::bind(&MotorControlCheckerNode::encoderCountCallback, this, _1))},
  encoderCounts{0, 0}
{
}

void MotorControlCheckerNode::changeMotorControlNodeToState(std::uint8_t transition)
{
  auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
  request->transition.id = transition;
  auto result = changeStateClient->async_send_request(request);
}

void MotorControlCheckerNode::encoderCountCallback(const HalMotorControlMsg_t & msg)
{
  encoderCounts.at(0) = msg.motor_left_encoder_count;
  encoderCounts.at(1) = msg.motor_right_encoder_count;
}

TEST_F(MotorControlTest, configureMotor)
{
  std::function<void()> configureMotor = std::bind(
    &MotorControl::configureMotor, motorControl);

  auto future = std::async(
    std::launch::async,
    configureMotor);

  auto status = executor.spin_until_future_complete(future);

  if (status == rclcpp::FutureReturnCode::SUCCESS) {
    ASSERT_EQ(get_mode(pigpioDummy->piHandle, MOTOR_LEFT_ENCODER_CH_A_GPIO), INPUT);
    ASSERT_EQ(get_mode(pigpioDummy->piHandle, MOTOR_LEFT_ENCODER_CH_B_GPIO), INPUT);
    ASSERT_EQ(get_mode(pigpioDummy->piHandle, MOTOR_RIGHT_ENCODER_CH_A_GPIO), INPUT);
    ASSERT_EQ(get_mode(pigpioDummy->piHandle, MOTOR_RIGHT_ENCODER_CH_B_GPIO), INPUT);
    ASSERT_EQ(get_mode(pigpioDummy->piHandle, MOTOR_LEFT_PWM_A_GPIO), OUTPUT);
    ASSERT_EQ(get_mode(pigpioDummy->piHandle, MOTOR_LEFT_PWM_B_GPIO), OUTPUT);
    ASSERT_EQ(get_mode(pigpioDummy->piHandle, MOTOR_RIGHT_PWM_A_GPIO), OUTPUT);
    ASSERT_EQ(get_mode(pigpioDummy->piHandle, MOTOR_RIGHT_PWM_B_GPIO), OUTPUT);
    ASSERT_EQ(
      get_PWM_frequency(pigpioDummy->piHandle, MOTOR_LEFT_PWM_A_GPIO), MOTOR_PWM_FREQUENCY);
    ASSERT_EQ(
      get_PWM_frequency(pigpioDummy->piHandle, MOTOR_LEFT_PWM_B_GPIO), MOTOR_PWM_FREQUENCY);
    ASSERT_EQ(
      get_PWM_frequency(pigpioDummy->piHandle, MOTOR_RIGHT_PWM_A_GPIO), MOTOR_PWM_FREQUENCY);
    ASSERT_EQ(
      get_PWM_frequency(pigpioDummy->piHandle, MOTOR_RIGHT_PWM_B_GPIO), MOTOR_PWM_FREQUENCY);
  } else {
    FAIL();
  }
}

TEST_F(MotorControlTest, setPwmLeftForward)
{
  std::function<void()> configureMotor = std::bind(
    &MotorControl::configureMotor, motorControl);

  auto future = std::async(
    std::launch::async,
    configureMotor);

  auto status = executor.spin_until_future_complete(future);

  if (status != rclcpp::FutureReturnCode::SUCCESS) {
    FAIL();
  }

  motorControl->setPwmLeft(20, FORWARD);
  executor.spin_some();
  executor.spin_some();

  ASSERT_EQ(get_PWM_dutycycle(pigpioDummy->piHandle, MOTOR_LEFT_PWM_A_GPIO), 20);
  ASSERT_EQ(get_PWM_dutycycle(pigpioDummy->piHandle, MOTOR_LEFT_PWM_B_GPIO), 20);
}

TEST_F(MotorControlTest, setPwmLeftBackward)
{
  std::function<void()> configureMotor = std::bind(
    &MotorControl::configureMotor, motorControl);

  auto future = std::async(
    std::launch::async,
    configureMotor);

  auto status = executor.spin_until_future_complete(future);

  if (status != rclcpp::FutureReturnCode::SUCCESS) {
    FAIL();
  }

  motorControl->setPwmLeft(20, BACKWARD);
  executor.spin_some();
  executor.spin_some();

  ASSERT_EQ(get_PWM_dutycycle(pigpioDummy->piHandle, MOTOR_LEFT_PWM_A_GPIO), 0);
  ASSERT_EQ(get_PWM_dutycycle(pigpioDummy->piHandle, MOTOR_LEFT_PWM_B_GPIO), 20);
}

TEST_F(MotorControlTest, setPwmRightForward)
{
  std::function<void()> configureMotor = std::bind(
    &MotorControl::configureMotor, motorControl);

  auto future = std::async(
    std::launch::async,
    configureMotor);

  auto status = executor.spin_until_future_complete(future);

  if (status != rclcpp::FutureReturnCode::SUCCESS) {
    FAIL();
  }

  motorControl->setPwmRight(20, FORWARD);
  executor.spin_some();
  executor.spin_some();

  ASSERT_EQ(get_PWM_dutycycle(pigpioDummy->piHandle, MOTOR_RIGHT_PWM_A_GPIO), 20);
  ASSERT_EQ(get_PWM_dutycycle(pigpioDummy->piHandle, MOTOR_RIGHT_PWM_B_GPIO), 20);
}

TEST_F(MotorControlTest, setPwmRightBackward)
{
  std::function<void()> configureMotor = std::bind(
    &MotorControl::configureMotor, motorControl);

  auto future = std::async(
    std::launch::async,
    configureMotor);

  auto status = executor.spin_until_future_complete(future);

  if (status != rclcpp::FutureReturnCode::SUCCESS) {
    FAIL();
  }

  motorControl->setPwmRight(20, BACKWARD);
  executor.spin_some();
  executor.spin_some();

  ASSERT_EQ(get_PWM_dutycycle(pigpioDummy->piHandle, MOTOR_RIGHT_PWM_A_GPIO), 0);
  ASSERT_EQ(get_PWM_dutycycle(pigpioDummy->piHandle, MOTOR_RIGHT_PWM_B_GPIO), 20);
}

TEST_F(MotorControlTest, encoderCountCallbackAndPublishMessageSuccess)
{
  std::vector<uint8_t> motors{MOTOR_LEFT, MOTOR_RIGHT};
  std::vector<int32_t> encodersCounts{-10, 10};

  auto message = HalPigpioEncoderCountMsg_t();

  message.motor_id = motors;
  message.encoder_count = encodersCounts;
  const HalPigpioEncoderCountMsg_t & messageToSend = message;

  motorControl->pigpioEncoderCountCallback(messageToSend);
  executor.spin_some();

  motorControl->activatePublisher();
  executor.spin_some();

  motorControl->publishMessage();
  executor.spin_some();

  ASSERT_EQ(motorControlChecker->encoderCounts.at(0), 10);
  ASSERT_EQ(motorControlChecker->encoderCounts.at(1), 10);
}

TEST_F(MotorControlTest, encoderCountCallbackAndPublishMessageFailure)
{
  std::vector<uint8_t> motors{MOTOR_LEFT, BAD_MOTOR_ID};
  std::vector<int32_t> encodersCounts{-10, 10};

  auto message = HalPigpioEncoderCountMsg_t();

  message.motor_id = motors;
  message.encoder_count = encodersCounts;
  const HalPigpioEncoderCountMsg_t & messageToSend = message;

  motorControl->pigpioEncoderCountCallback(messageToSend);
  executor.spin_some();

  motorControl->activatePublisher();
  executor.spin_some();

  motorControl->publishMessage();
  executor.spin_some();

  ASSERT_EQ(motorControlChecker->encoderCounts.at(0), 10);
  ASSERT_EQ(motorControlChecker->encoderCounts.at(1), 0);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  auto result = RUN_ALL_TESTS();

  rclcpp::shutdown();
  return result;
}
