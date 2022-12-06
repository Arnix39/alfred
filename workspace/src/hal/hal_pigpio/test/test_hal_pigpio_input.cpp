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
  hal_pigpioGpioSet(GOOD_GPIO, pigioChecker->setGpioHighClient, &executor);
  ASSERT_EQ(pigioChecker->readGpioAndCheckLevel(GOOD_GPIO, PI_HIGH, &executor), true);
}

TEST_F(PigpioTest, ReadGpioFailure)
{
  ASSERT_EQ(pigioChecker->readGpioAndCheckLevel(BAD_GPIO, PI_LOW, &executor), false);
}

TEST_F(PigpioTest, SetCallbackSuccess)
{
  hal_pigpioGpioSet(GOOD_GPIO, pigioChecker->setInputModeClient, &executor);
  ASSERT_EQ(pigioChecker->setCallback(GOOD_GPIO, AS_EITHER_EDGE, &executor), true);
}

TEST_F(PigpioTest, SetCallbackFailure)
{
  hal_pigpioGpioSet(GOOD_GPIO, pigioChecker->setOutputModeClient, &executor);
  ASSERT_EQ(pigioChecker->setCallback(GOOD_GPIO, AS_EITHER_EDGE, &executor), false);
}

TEST_F(PigpioTest, SetEncoderCallbackSuccess)
{
  hal_pigpioGpioSet(GOOD_GPIO, pigioChecker->setInputModeClient, &executor);
  ASSERT_EQ(
    pigioChecker->setEncoderCallback(GOOD_GPIO, AS_EITHER_EDGE, MOTOR_ID_1, &executor),
    true);
}

TEST_F(PigpioTest, SetEncoderCallbackFailure)
{
  hal_pigpioGpioSet(GOOD_GPIO, pigioChecker->setOutputModeClient, &executor);
  ASSERT_EQ(
    pigioChecker->setEncoderCallback(GOOD_GPIO, AS_EITHER_EDGE, MOTOR_ID_1, &executor),
    false);
}

TEST_F(PigpioTest, SetMotorDirectionSuccess)
{
  hal_pigpioGpioSet(GOOD_GPIO, pigioChecker->setInputModeClient, &executor);
  pigioChecker->setEncoderCallback(GOOD_GPIO, AS_EITHER_EDGE, MOTOR_ID_1, &executor);
  ASSERT_EQ(pigioChecker->setMotorDirection(FORWARD, MOTOR_ID_1, &executor), true);
}

TEST_F(PigpioTest, SetMotorDirectionFailure)
{
  hal_pigpioGpioSet(GOOD_GPIO, pigioChecker->setOutputModeClient, &executor);
  pigioChecker->setEncoderCallback(GOOD_GPIO, AS_EITHER_EDGE, MOTOR_ID_1, &executor);
  ASSERT_EQ(pigioChecker->setMotorDirection(FORWARD, MOTOR_ID_2, &executor), false);
}

TEST_F(PigpioTest, PublishRisingEdgeChangeMessage)
{
  pigpio->gpioEdgeChangeCallback(0, GOOD_GPIO, RISING_EDGE, 1000);

  executor.spin_some();

  ASSERT_EQ(pigioChecker->edgeChangeMsg_gpioId, GOOD_GPIO);
  ASSERT_EQ(pigioChecker->edgeChangeMsg_edgeChangeType, RISING_EDGE);
  ASSERT_EQ(pigioChecker->edgeChangeMsg_timeSinceBoot_us, 1000);
}

TEST_F(PigpioTest, PublishFallingEdgeChangeMessage)
{
  pigpio->gpioEdgeChangeCallback(0, GOOD_GPIO, FALLING_EDGE, 1000);

  executor.spin_some();

  ASSERT_EQ(pigioChecker->edgeChangeMsg_gpioId, GOOD_GPIO);
  ASSERT_EQ(pigioChecker->edgeChangeMsg_edgeChangeType, FALLING_EDGE);
  ASSERT_EQ(pigioChecker->edgeChangeMsg_timeSinceBoot_us, 1000);
}

TEST_F(PigpioTest, PublishPositiveEncoderCountMessage)
{
  hal_pigpioGpioSet(GOOD_GPIO, pigioChecker->setInputModeClient, &executor);
  pigioChecker->setEncoderCallback(GOOD_GPIO, AS_EITHER_EDGE, MOTOR_ID_1, &executor);
  pigioChecker->setMotorDirection(FORWARD, MOTOR_ID_1, &executor);
  pigpio->gpioEncoderEdgeChangeCallback(0, GOOD_GPIO, RISING_EDGE, 1000);
  pigpio->publishEncoderCount();
  executor.spin_some();

  auto motorIndex = pigioChecker->motorsEC.find(MOTOR_ID_1);
  ASSERT_EQ((motorIndex != pigioChecker->motorsEC.end()), true);
  ASSERT_EQ(motorIndex->second, 1);
}

TEST_F(PigpioTest, PublishPositiveEncoderCountMessageTwice)
{
  hal_pigpioGpioSet(GOOD_GPIO, pigioChecker->setInputModeClient, &executor);
  pigioChecker->setEncoderCallback(GOOD_GPIO, AS_EITHER_EDGE, MOTOR_ID_1, &executor);
  pigioChecker->setMotorDirection(FORWARD, MOTOR_ID_1, &executor);
  pigpio->gpioEncoderEdgeChangeCallback(0, GOOD_GPIO, RISING_EDGE, 1000);
  pigpio->publishEncoderCount();
  executor.spin_some();

  auto motorIndex = pigioChecker->motorsEC.find(MOTOR_ID_1);
  ASSERT_EQ((motorIndex != pigioChecker->motorsEC.end()), true);
  ASSERT_EQ(motorIndex->second, 1);

  pigpio->gpioEncoderEdgeChangeCallback(0, GOOD_GPIO, RISING_EDGE, 1000);
  pigpio->publishEncoderCount();
  executor.spin_some();

  motorIndex = pigioChecker->motorsEC.find(MOTOR_ID_1);
  ASSERT_EQ((motorIndex != pigioChecker->motorsEC.end()), true);
  ASSERT_EQ(motorIndex->second, 2);
}

TEST_F(PigpioTest, PublishNegativeEncoderCountMessage)
{
  hal_pigpioGpioSet(GOOD_GPIO, pigioChecker->setInputModeClient, &executor);
  pigioChecker->setEncoderCallback(GOOD_GPIO, AS_EITHER_EDGE, MOTOR_ID_2, &executor);
  pigioChecker->setMotorDirection(BACKWARD, MOTOR_ID_2, &executor);
  pigpio->gpioEncoderEdgeChangeCallback(0, GOOD_GPIO, RISING_EDGE, 1000);
  pigpio->publishEncoderCount();
  executor.spin_some();

  auto motorIndex = pigioChecker->motorsEC.find(MOTOR_ID_2);
  ASSERT_EQ((motorIndex != pigioChecker->motorsEC.end()), true);
  ASSERT_EQ(motorIndex->second, -1);
}

TEST_F(PigpioTest, PublishNegativeEncoderCountMessageTwice)
{
  hal_pigpioGpioSet(GOOD_GPIO, pigioChecker->setInputModeClient, &executor);
  pigioChecker->setEncoderCallback(GOOD_GPIO, AS_EITHER_EDGE, MOTOR_ID_2, &executor);
  pigioChecker->setMotorDirection(BACKWARD, MOTOR_ID_2, &executor);
  pigpio->gpioEncoderEdgeChangeCallback(0, GOOD_GPIO, RISING_EDGE, 1000);
  pigpio->publishEncoderCount();
  executor.spin_some();

  auto motorIndex = pigioChecker->motorsEC.find(MOTOR_ID_2);
  ASSERT_EQ((motorIndex != pigioChecker->motorsEC.end()), true);
  ASSERT_EQ(motorIndex->second, -1);

  pigpio->gpioEncoderEdgeChangeCallback(0, GOOD_GPIO, RISING_EDGE, 1000);
  pigpio->publishEncoderCount();
  executor.spin_some();

  motorIndex = pigioChecker->motorsEC.find(MOTOR_ID_2);
  ASSERT_EQ((motorIndex != pigioChecker->motorsEC.end()), true);
  ASSERT_EQ(motorIndex->second, -2);
}

TEST_F(PigpioTest, PublishEncoderCountMessageTwoMotors)
{
  hal_pigpioGpioSet(GOOD_GPIO, pigioChecker->setInputModeClient, &executor);
  hal_pigpioGpioSet(GOOD_GPIO_2, pigioChecker->setInputModeClient, &executor);

  pigioChecker->setEncoderCallback(GOOD_GPIO, AS_EITHER_EDGE, MOTOR_ID_1, &executor);
  pigioChecker->setEncoderCallback(GOOD_GPIO_2, AS_EITHER_EDGE, MOTOR_ID_2, &executor);
  pigioChecker->setMotorDirection(FORWARD, MOTOR_ID_1, &executor);
  pigioChecker->setMotorDirection(BACKWARD, MOTOR_ID_2, &executor);
  pigpio->gpioEncoderEdgeChangeCallback(0, GOOD_GPIO_2, RISING_EDGE, 1000);
  pigpio->publishEncoderCount();
  executor.spin_some();

  auto motorIndex = pigioChecker->motorsEC.find(MOTOR_ID_2);
  ASSERT_EQ((motorIndex != pigioChecker->motorsEC.end()), true);
  ASSERT_EQ(motorIndex->second, -1);

  motorIndex = pigioChecker->motorsEC.find(MOTOR_ID_1);
  ASSERT_EQ((motorIndex != pigioChecker->motorsEC.end()), true);
  ASSERT_EQ(motorIndex->second, 0);
}

TEST_F(PigpioTest, PublishEncoderCountMessageOneMotorTwoGpios)
{
  hal_pigpioGpioSet(GOOD_GPIO, pigioChecker->setInputModeClient, &executor);
  hal_pigpioGpioSet(GOOD_GPIO_2, pigioChecker->setInputModeClient, &executor);

  pigioChecker->setEncoderCallback(GOOD_GPIO, AS_EITHER_EDGE, MOTOR_ID_2, &executor);
  pigioChecker->setEncoderCallback(GOOD_GPIO_2, AS_EITHER_EDGE, MOTOR_ID_2, &executor);
  pigioChecker->setMotorDirection(FORWARD, MOTOR_ID_2, &executor);
  pigpio->gpioEncoderEdgeChangeCallback(0, GOOD_GPIO, RISING_EDGE, 1000);
  pigpio->gpioEncoderEdgeChangeCallback(0, GOOD_GPIO_2, RISING_EDGE, 1000);
  pigpio->publishEncoderCount();
  executor.spin_some();

  auto motorIndex = pigioChecker->motorsEC.find(MOTOR_ID_2);
  ASSERT_EQ((motorIndex != pigioChecker->motorsEC.end()), true);
  ASSERT_EQ(motorIndex->second, 2);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  auto result = RUN_ALL_TESTS();

  rclcpp::shutdown();
  return result;
}
