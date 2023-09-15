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
  hal_pigpioGpioSet(GOOD_GPIO, pigpioChecker->setGpioHighClient, &executor);
  ASSERT_EQ(pigpioChecker->readGpioAndCheckLevel(GOOD_GPIO, PI_HIGH, &executor), true);
}

TEST_F(PigpioTest, ReadGpioFailure)
{
  ASSERT_EQ(pigpioChecker->readGpioAndCheckLevel(BAD_GPIO, PI_LOW, &executor), false);
}

TEST_F(PigpioTest, SetCallbackSuccess)
{
  hal_pigpioGpioSet(GOOD_GPIO, pigpioChecker->setInputModeClient, &executor);
  ASSERT_EQ(
    pigpioChecker->setCallback(GOOD_GPIO, EdgeChangeConfiguration::asEitherEdge, &executor),
    true);
}

TEST_F(PigpioTest, SetCallbackFailure)
{
  hal_pigpioGpioSet(GOOD_GPIO, pigpioChecker->setOutputModeClient, &executor);
  ASSERT_EQ(
    pigpioChecker->setCallback(GOOD_GPIO, EdgeChangeConfiguration::asEitherEdge, &executor),
    false);
}

TEST_F(PigpioTest, SetEncoderCallbackSuccess)
{
  hal_pigpioGpioSet(GOOD_GPIO, pigpioChecker->setInputModeClient, &executor);
  ASSERT_EQ(
    pigpioChecker->setEncoderCallback(
      GOOD_GPIO, EdgeChangeConfiguration::asEitherEdge, MOTOR_ID_1, EncoderChannel::channelA,
      &executor),
    true);
}

TEST_F(PigpioTest, SetEncoderCallbackFailure)
{
  hal_pigpioGpioSet(GOOD_GPIO, pigpioChecker->setOutputModeClient, &executor);
  ASSERT_EQ(
    pigpioChecker->setEncoderCallback(
      GOOD_GPIO, EdgeChangeConfiguration::asEitherEdge, MOTOR_ID_1, EncoderChannel::channelA,
      &executor),
    false);
}

TEST_F(PigpioTest, PublishRisingEdgeChangeMessage)
{
  pigpio->gpioEdgeChangeCallback(
    0, GOOD_GPIO, static_cast<unsigned>(EdgeChangeType::rising), 1000);

  executor.spin_some();

  ASSERT_EQ(pigpioChecker->edgeChangeMsg_gpioId, GOOD_GPIO);
  ASSERT_EQ(pigpioChecker->edgeChangeMsg_edgeChangeType, EdgeChangeType::rising);
  ASSERT_EQ(pigpioChecker->edgeChangeMsg_timeSinceBoot_us, 1000);
}

TEST_F(PigpioTest, PublishFallingEdgeChangeMessage)
{
  pigpio->gpioEdgeChangeCallback(
    0, GOOD_GPIO, static_cast<unsigned>(EdgeChangeType::falling), 1000);

  executor.spin_some();

  ASSERT_EQ(pigpioChecker->edgeChangeMsg_gpioId, GOOD_GPIO);
  ASSERT_EQ(pigpioChecker->edgeChangeMsg_edgeChangeType, EdgeChangeType::falling);
  ASSERT_EQ(pigpioChecker->edgeChangeMsg_timeSinceBoot_us, 1000);
}

TEST_F(PigpioTest, ConputeDirectionForward)
{
  ASSERT_EQ(
    pigpio->computeDirection(
      EncoderChannel::channelA, EncoderChannel::channelB,
      EdgeChangeType::rising, EdgeChangeType::rising),
    MotorDirection::forward);
  ASSERT_EQ(
    pigpio->computeDirection(
      EncoderChannel::channelA, EncoderChannel::channelB,
      EdgeChangeType::falling, EdgeChangeType::falling),
    MotorDirection::forward);
  ASSERT_EQ(
    pigpio->computeDirection(
      EncoderChannel::channelB, EncoderChannel::channelA,
      EdgeChangeType::falling, EdgeChangeType::rising),
    MotorDirection::forward);
  ASSERT_EQ(
    pigpio->computeDirection(
      EncoderChannel::channelB, EncoderChannel::channelA,
      EdgeChangeType::rising, EdgeChangeType::falling),
    MotorDirection::forward);
}

TEST_F(PigpioTest, ConputeDirectionBackward)
{
  ASSERT_EQ(
    pigpio->computeDirection(
      EncoderChannel::channelB, EncoderChannel::channelA,
      EdgeChangeType::rising, EdgeChangeType::rising),
    MotorDirection::backward);
  ASSERT_EQ(
    pigpio->computeDirection(
      EncoderChannel::channelB, EncoderChannel::channelA,
      EdgeChangeType::falling, EdgeChangeType::falling),
    MotorDirection::backward);
  ASSERT_EQ(
    pigpio->computeDirection(
      EncoderChannel::channelA, EncoderChannel::channelB,
      EdgeChangeType::falling, EdgeChangeType::rising),
    MotorDirection::backward);
  ASSERT_EQ(
    pigpio->computeDirection(
      EncoderChannel::channelA, EncoderChannel::channelB,
      EdgeChangeType::falling, EdgeChangeType::rising),
    MotorDirection::backward);
}

TEST_F(PigpioTest, ConputeDirectionUndefined)
{
  ASSERT_EQ(
    pigpio->computeDirection(
      EncoderChannel::channelA, EncoderChannel::channelA,
      EdgeChangeType::rising, EdgeChangeType::rising),
    MotorDirection::undefined);
  ASSERT_EQ(
    pigpio->computeDirection(
      EncoderChannel::channelB, EncoderChannel::channelB,
      EdgeChangeType::rising, EdgeChangeType::rising),
    MotorDirection::undefined);
  ASSERT_EQ(
    pigpio->computeDirection(
      EncoderChannel::undefined, EncoderChannel::channelB,
      EdgeChangeType::rising, EdgeChangeType::rising),
    MotorDirection::undefined);
  ASSERT_EQ(
    pigpio->computeDirection(
      EncoderChannel::undefined, EncoderChannel::channelA,
      EdgeChangeType::rising, EdgeChangeType::rising),
    MotorDirection::undefined);
}

TEST_F(PigpioTest, PublishPositiveEncoderCountMessage)
{
  hal_pigpioGpioSet(CHANNEL_A_MOTOR_1, pigpioChecker->setInputModeClient, &executor);
  hal_pigpioGpioSet(CHANNEL_B_MOTOR_1, pigpioChecker->setInputModeClient, &executor);

  pigpioChecker->setEncoderCallback(
    CHANNEL_A_MOTOR_1, EdgeChangeConfiguration::asEitherEdge, MOTOR_ID_1, EncoderChannel::channelA,
    &executor);
  pigpioChecker->setEncoderCallback(
    CHANNEL_B_MOTOR_1, EdgeChangeConfiguration::asEitherEdge, MOTOR_ID_1, EncoderChannel::channelB,
    &executor);

  /* Channel A rising first then channel B */
  pigpio->gpioEncoderEdgeChangeCallback(
    0, CHANNEL_A_MOTOR_1, static_cast<unsigned>(EdgeChangeType::rising), 1000);
  pigpio->gpioEncoderEdgeChangeCallback(
    0, CHANNEL_B_MOTOR_1, static_cast<unsigned>(EdgeChangeType::rising), 2000);
  pigpio->publishEncoderCount();
  executor.spin_some();

  auto motorIndex = pigpioChecker->motorsEC.find(MOTOR_ID_1);
  ASSERT_EQ((motorIndex != pigpioChecker->motorsEC.end()), true);
  ASSERT_EQ(motorIndex->second, 1);
}

TEST_F(PigpioTest, PublishPositiveEncoderCountMessageTwice)
{
  hal_pigpioGpioSet(CHANNEL_A_MOTOR_1, pigpioChecker->setInputModeClient, &executor);
  hal_pigpioGpioSet(CHANNEL_B_MOTOR_1, pigpioChecker->setInputModeClient, &executor);

  pigpioChecker->setEncoderCallback(
    CHANNEL_A_MOTOR_1, EdgeChangeConfiguration::asEitherEdge, MOTOR_ID_1, EncoderChannel::channelA,
    &executor);
  pigpioChecker->setEncoderCallback(
    CHANNEL_B_MOTOR_1, EdgeChangeConfiguration::asEitherEdge, MOTOR_ID_1, EncoderChannel::channelB,
    &executor);

  /* Channel A rising first then channel B */
  pigpio->gpioEncoderEdgeChangeCallback(
    0, CHANNEL_A_MOTOR_1, static_cast<unsigned>(EdgeChangeType::rising), 1000);
  pigpio->gpioEncoderEdgeChangeCallback(
    0, CHANNEL_B_MOTOR_1, static_cast<unsigned>(EdgeChangeType::rising), 2000);
  pigpio->publishEncoderCount();
  executor.spin_some();

  auto motorIndex = pigpioChecker->motorsEC.find(MOTOR_ID_1);
  ASSERT_EQ((motorIndex != pigpioChecker->motorsEC.end()), true);
  ASSERT_EQ(motorIndex->second, 1);

  /* Channel A falling */
  pigpio->gpioEncoderEdgeChangeCallback(
    0, CHANNEL_A_MOTOR_1, static_cast<unsigned>(EdgeChangeType::falling), 3000);
  pigpio->publishEncoderCount();
  executor.spin_some();

  motorIndex = pigpioChecker->motorsEC.find(MOTOR_ID_1);
  ASSERT_EQ((motorIndex != pigpioChecker->motorsEC.end()), true);
  ASSERT_EQ(motorIndex->second, 2);
}

TEST_F(PigpioTest, PublishNegativeEncoderCountMessage)
{
  hal_pigpioGpioSet(CHANNEL_A_MOTOR_2, pigpioChecker->setInputModeClient, &executor);
  hal_pigpioGpioSet(CHANNEL_B_MOTOR_2, pigpioChecker->setInputModeClient, &executor);

  pigpioChecker->setEncoderCallback(
    CHANNEL_A_MOTOR_2, EdgeChangeConfiguration::asEitherEdge, MOTOR_ID_2, EncoderChannel::channelA,
    &executor);
  pigpioChecker->setEncoderCallback(
    CHANNEL_B_MOTOR_2, EdgeChangeConfiguration::asEitherEdge, MOTOR_ID_2, EncoderChannel::channelB,
    &executor);

  /* Channel B rising first then channel A */
  pigpio->gpioEncoderEdgeChangeCallback(
    0, CHANNEL_B_MOTOR_2, static_cast<unsigned>(EdgeChangeType::rising), 1000);
  pigpio->gpioEncoderEdgeChangeCallback(
    0, CHANNEL_A_MOTOR_2, static_cast<unsigned>(EdgeChangeType::rising), 2000);
  pigpio->publishEncoderCount();
  executor.spin_some();

  auto motorIndex = pigpioChecker->motorsEC.find(MOTOR_ID_2);
  ASSERT_EQ((motorIndex != pigpioChecker->motorsEC.end()), true);
  ASSERT_EQ(motorIndex->second, -1);
}

TEST_F(PigpioTest, PublishNegativeEncoderCountMessageTwice)
{
  hal_pigpioGpioSet(CHANNEL_A_MOTOR_2, pigpioChecker->setInputModeClient, &executor);
  hal_pigpioGpioSet(CHANNEL_B_MOTOR_2, pigpioChecker->setInputModeClient, &executor);

  pigpioChecker->setEncoderCallback(
    CHANNEL_A_MOTOR_2, EdgeChangeConfiguration::asEitherEdge, MOTOR_ID_2, EncoderChannel::channelA,
    &executor);
  pigpioChecker->setEncoderCallback(
    CHANNEL_B_MOTOR_2, EdgeChangeConfiguration::asEitherEdge, MOTOR_ID_2, EncoderChannel::channelB,
    &executor);

  /* Channel B rising first then channel A */
  pigpio->gpioEncoderEdgeChangeCallback(
    0, CHANNEL_B_MOTOR_2, static_cast<unsigned>(EdgeChangeType::rising), 1000);
  pigpio->gpioEncoderEdgeChangeCallback(
    0, CHANNEL_A_MOTOR_2, static_cast<unsigned>(EdgeChangeType::rising), 2000);
  pigpio->publishEncoderCount();
  executor.spin_some();

  auto motorIndex = pigpioChecker->motorsEC.find(MOTOR_ID_2);
  ASSERT_EQ((motorIndex != pigpioChecker->motorsEC.end()), true);
  ASSERT_EQ(motorIndex->second, -1);

  /* Channel B falling */
  pigpio->gpioEncoderEdgeChangeCallback(
    0, CHANNEL_B_MOTOR_2, static_cast<unsigned>(EdgeChangeType::falling), 3000);
  pigpio->publishEncoderCount();
  executor.spin_some();

  motorIndex = pigpioChecker->motorsEC.find(MOTOR_ID_2);
  ASSERT_EQ((motorIndex != pigpioChecker->motorsEC.end()), true);
  ASSERT_EQ(motorIndex->second, -2);
}

TEST_F(PigpioTest, PublishEncoderCountMessageTwoMotors)
{
  hal_pigpioGpioSet(CHANNEL_A_MOTOR_1, pigpioChecker->setInputModeClient, &executor);
  hal_pigpioGpioSet(CHANNEL_B_MOTOR_1, pigpioChecker->setInputModeClient, &executor);
  hal_pigpioGpioSet(CHANNEL_A_MOTOR_2, pigpioChecker->setInputModeClient, &executor);
  hal_pigpioGpioSet(CHANNEL_B_MOTOR_2, pigpioChecker->setInputModeClient, &executor);

  pigpioChecker->setEncoderCallback(
    CHANNEL_A_MOTOR_1, EdgeChangeConfiguration::asEitherEdge, MOTOR_ID_1, EncoderChannel::channelA,
    &executor);
  pigpioChecker->setEncoderCallback(
    CHANNEL_B_MOTOR_1, EdgeChangeConfiguration::asEitherEdge, MOTOR_ID_1, EncoderChannel::channelB,
    &executor);
  pigpioChecker->setEncoderCallback(
    CHANNEL_A_MOTOR_2, EdgeChangeConfiguration::asEitherEdge, MOTOR_ID_2, EncoderChannel::channelA,
    &executor);
  pigpioChecker->setEncoderCallback(
    CHANNEL_B_MOTOR_2, EdgeChangeConfiguration::asEitherEdge, MOTOR_ID_2, EncoderChannel::channelB,
    &executor);

  /* Channel A rising first then channel B for motor 1*/
  pigpio->gpioEncoderEdgeChangeCallback(
    0, CHANNEL_A_MOTOR_1, static_cast<unsigned>(EdgeChangeType::rising), 1000);
  pigpio->gpioEncoderEdgeChangeCallback(
    0, CHANNEL_B_MOTOR_1, static_cast<unsigned>(EdgeChangeType::rising), 2000);

  /* Channel B rising first then channel A for motor 2 */
  pigpio->gpioEncoderEdgeChangeCallback(
    0, CHANNEL_B_MOTOR_2, static_cast<unsigned>(EdgeChangeType::rising), 3000);
  pigpio->gpioEncoderEdgeChangeCallback(
    0, CHANNEL_A_MOTOR_2, static_cast<unsigned>(EdgeChangeType::rising), 4000);

  pigpio->publishEncoderCount();
  executor.spin_some();

  auto motorIndex = pigpioChecker->motorsEC.find(MOTOR_ID_2);
  ASSERT_EQ((motorIndex != pigpioChecker->motorsEC.end()), true);
  ASSERT_EQ(motorIndex->second, -1);

  motorIndex = pigpioChecker->motorsEC.find(MOTOR_ID_1);
  ASSERT_EQ((motorIndex != pigpioChecker->motorsEC.end()), true);
  ASSERT_EQ(motorIndex->second, 1);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  auto result = RUN_ALL_TESTS();

  rclcpp::shutdown();
  return result;
}
