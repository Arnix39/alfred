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

#include "hal_motor_tests.hpp"

using namespace std::placeholders;

HalPigpioDummyNode::HalPigpioDummyNode()
: rclcpp::Node("hal_pigpio_dummy_node"),
  piHandle(pigpio_start(NULL, NULL)),
  callbackId(pigif_bad_callback),
  setInputModeService(this->create_service<HalPigpioSetInputMode_t>(
      "hal_pigpioSetInputMode", std::bind(&HalPigpioDummyNode::setInputMode, this, _1, _2))),
  setOutputModeService(this->create_service<HalPigpioSetOutputMode_t>(
      "hal_pigpioSetOutputMode", std::bind(&HalPigpioDummyNode::setOutputMode, this, _1, _2))),
  setPwmDutycycleService(this->create_service<HalPigpioSetPwmDutycycle_t>(
      "hal_pigpioSetPwmDutycycle", std::bind(&HalPigpioDummyNode::setPwmDutycycle, this, _1, _2))),
  setPwmFrequencyService(this->create_service<HalPigpioSetPwmFrequency_t>(
      "hal_pigpioSetPwmFrequency", std::bind(&HalPigpioDummyNode::setPwmFrequency, this, _1, _2))),
  setEncoderCallbackService(this->create_service<HalPigpioSetEncoderCallback_t>(
      "hal_pigpioSetEncoderCallback",
      std::bind(&HalPigpioDummyNode::setEncoderCallback, this, _1, _2))),
  setMotorDirectionService(this->create_service<HalPigpioSetMotorDirection_t>(
      "hal_pigpioSetMotorDirection",
      std::bind(&HalPigpioDummyNode::setMotorDirection, this, _1, _2)))
{
}

HalPigpioDummyNode::~HalPigpioDummyNode()
{
  if (callbackId >= 0) {
    callback_cancel(callbackId);
  }

  pigpio_stop(piHandle);
}

void HalPigpioDummyNode::setInputMode(
  const std::shared_ptr<HalPigpioSetInputMode_t::Request> request,
  std::shared_ptr<HalPigpioSetInputMode_t::Response> response)
{
  if (set_mode(piHandle, request->gpio_id, PI_INPUT) == 0) {
    response->has_succeeded = true;
  } else {
    response->has_succeeded = false;
  }
}

void HalPigpioDummyNode::setOutputMode(
  const std::shared_ptr<HalPigpioSetOutputMode_t::Request> request,
  std::shared_ptr<HalPigpioSetOutputMode_t::Response> response)
{
  if (set_mode(piHandle, request->gpio_id, PI_OUTPUT) == 0) {
    response->has_succeeded = true;
  } else {
    response->has_succeeded = false;
  }
}

void HalPigpioDummyNode::setPwmDutycycle(
  const std::shared_ptr<HalPigpioSetPwmDutycycle_t::Request> request,
  std::shared_ptr<HalPigpioSetPwmDutycycle_t::Response> response)
{
  if (set_PWM_dutycycle(piHandle, request->gpio_id, request->dutycycle) == 0) {
    response->has_succeeded = true;
  } else {
    response->has_succeeded = false;
  }
}

void HalPigpioDummyNode::setPwmFrequency(
  const std::shared_ptr<HalPigpioSetPwmFrequency_t::Request> request,
  std::shared_ptr<HalPigpioSetPwmFrequency_t::Response> response)
{
  if (set_PWM_frequency(piHandle, request->gpio_id, request->frequency) == 0) {
    response->has_succeeded = true;
  } else {
    response->has_succeeded = false;
  }
}

void gpioEncoderEdgeChangeCallback(
  int handle, unsigned gpioId, unsigned edgeChangeType,
  uint32_t timeSinceBoot_us)
{
  (void)handle;
  (void)edgeChangeType;
  (void)timeSinceBoot_us;
}

void HalPigpioDummyNode::setEncoderCallback(
  const std::shared_ptr<HalPigpioSetEncoderCallback_t::Request> request,
  std::shared_ptr<HalPigpioSetEncoderCallback_t::Response> response)
{
  response->callback_id = callback(
    piHandle, request->gpio_id, request->edge_change_type, gpioEncoderEdgeChangeCallback);
  if (response->callback_id >= 0) {
    response->has_succeeded = true;
    callbackId = response->callback_id;
  } else {
    response->has_succeeded = false;
  }
}

void HalPigpioDummyNode::setMotorDirection(
  const std::shared_ptr<HalPigpioSetMotorDirection_t::Request> request,
  std::shared_ptr<HalPigpioSetMotorDirection_t::Response> response)
{
  response->has_succeeded = true;
}

MotorNode::MotorNode()
: rclcpp::Node("hal_motor_node"),
  motorOk(
    GPIO_PWM_CHANNEL_A_M1, GPIO_PWM_CHANNEL_B_M1,
    GPIO_ENCODER_CHANNEL_A_M1, GPIO_ENCODER_CHANNEL_B_M1,
    MOTOR_ID_1)
{
}

MotorCheckerNode::MotorCheckerNode()
: rclcpp::Node("hal_motor_checker_node"),
  setPwmDutycycleClient(
    this->create_client<HalPigpioSetPwmDutycycle_t>("hal_pigpioSetPwmDutycycle")),
  setMotorDirectionClient(
    this->create_client<HalPigpioSetMotorDirection_t>("hal_pigpioSetMotorDirection")),
  setInputModeClient("setInputModeSyncClientMotorChecker_node"),
  setOutputModeClient("setOutputModeSyncClientMotorChecker_node"),
  setEncoderCallbackClient("setEncoderCallbackSyncClientMotorChecker_node"),
  setPwmFrequencyClient("setPwmFrequencySyncClientMotorChecker_node")
{
}

TEST_F(MotorTest, EncoderCountUtils)
{
  const uint32_t encoderCount = 1024;
  motor->motorOk.setEncoderCount(encoderCount);

  ASSERT_EQ(motor->motorOk.getEncoderCount(), encoderCount);
}

TEST_F(MotorTest, GetId)
{
  ASSERT_EQ(motor->motorOk.getId(), MOTOR_ID_1);
}

TEST_F(MotorTest, ConfigureGpios)
{
  std::function<void(
      setOutputModeSyncClientNode_t,
      setInputModeSyncClientNode_t,
      setEncoderCallbackSyncClientNode_t,
      setPwmFrequencySyncClientNode_t)> configureGpios = std::bind(
    &Motor::configureGpios, motor->motorOk, _1, _2, _3, _4);

  auto future = std::async(
    std::launch::async,
    configureGpios,
    motorChecker->setOutputModeClient,
    motorChecker->setInputModeClient,
    motorChecker->setEncoderCallbackClient,
    motorChecker->setPwmFrequencyClient);

  auto status = executor.spin_until_future_complete(future);

  if (status == rclcpp::FutureReturnCode::SUCCESS) {
    ASSERT_EQ(get_mode(pigpioDummy->piHandle, GPIO_ENCODER_CHANNEL_A_M1), INPUT);
    ASSERT_EQ(get_mode(pigpioDummy->piHandle, GPIO_ENCODER_CHANNEL_B_M1), INPUT);
    ASSERT_EQ(get_mode(pigpioDummy->piHandle, GPIO_PWM_CHANNEL_A_M1), OUTPUT);
    ASSERT_EQ(get_mode(pigpioDummy->piHandle, GPIO_PWM_CHANNEL_B_M1), OUTPUT);
    ASSERT_EQ(getPwmFrequency(GPIO_PWM_CHANNEL_A_M1), MOTOR_PWM_FREQUENCY);
    ASSERT_EQ(getPwmFrequency(GPIO_PWM_CHANNEL_B_M1), MOTOR_PWM_FREQUENCY);
  } else {
    FAIL();
  }
}

TEST_F(MotorTest, SetPwmDutyCycleAndDirectionForward)
{
  std::function<void(
      setOutputModeSyncClientNode_t,
      setInputModeSyncClientNode_t,
      setEncoderCallbackSyncClientNode_t,
      setPwmFrequencySyncClientNode_t)> configureGpios = std::bind(
    &Motor::configureGpios, motor->motorOk, _1, _2, _3, _4);

  auto future = std::async(
    std::launch::async,
    configureGpios,
    motorChecker->setOutputModeClient,
    motorChecker->setInputModeClient,
    motorChecker->setEncoderCallbackClient,
    motorChecker->setPwmFrequencyClient);

  auto status = executor.spin_until_future_complete(future);

  if (status != rclcpp::FutureReturnCode::SUCCESS) {
    FAIL();
  }

  motor->motorOk.setPwmDutyCycleAndDirection(
    motorChecker->setPwmDutycycleClient,
    20,
    motorChecker->setMotorDirectionClient,
    true);

  executor.spin_some();
  executor.spin_some();

  ASSERT_EQ(getPwmDutycycle(GPIO_PWM_CHANNEL_A_M1), 20);
  ASSERT_EQ(getPwmDutycycle(GPIO_PWM_CHANNEL_B_M1), 20);
}

TEST_F(MotorTest, SetPwmDutyCycleAndDirectionBackward)
{
  std::function<void(
      setOutputModeSyncClientNode_t,
      setInputModeSyncClientNode_t,
      setEncoderCallbackSyncClientNode_t,
      setPwmFrequencySyncClientNode_t)> configureGpios = std::bind(
    &Motor::configureGpios, motor->motorOk, _1, _2, _3, _4);

  auto future = std::async(
    std::launch::async,
    configureGpios,
    motorChecker->setOutputModeClient,
    motorChecker->setInputModeClient,
    motorChecker->setEncoderCallbackClient,
    motorChecker->setPwmFrequencyClient);

  auto status = executor.spin_until_future_complete(future);

  if (status != rclcpp::FutureReturnCode::SUCCESS) {
    FAIL();
  }

  motor->motorOk.setPwmDutyCycleAndDirection(
    motorChecker->setPwmDutycycleClient,
    20,
    motorChecker->setMotorDirectionClient,
    false);

  executor.spin_some();
  executor.spin_some();

  ASSERT_EQ(getPwmDutycycle(GPIO_PWM_CHANNEL_A_M1), 0);
  ASSERT_EQ(getPwmDutycycle(GPIO_PWM_CHANNEL_B_M1), 20);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  auto result = RUN_ALL_TESTS();

  rclcpp::shutdown();
  return result;
}
