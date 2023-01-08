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

#ifndef HAL_PIGPIO_TESTS_HPP_
#define HAL_PIGPIO_TESTS_HPP_

#include <memory>
#include <map>
#include <vector>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"

#include "hal_pigpio.hpp"

#define GOOD_GPIO GPIO2
#define GOOD_GPIO_2 GPIO3
#define BAD_GPIO GPIO1
#define MOTOR_ID_1 0
#define MOTOR_ID_2 1
#define FORWARD true
#define BACKWARD false
#define I2C_BAD_ADDRESS 0x66
#define I2C_GOOD_ADDRESS MPU6050_I2C_ADDRESS
#define I2C_GOOD_ADDRESS_2 0x67
#define I2C_GOOD_BUS_1 0
#define I2C_GOOD_BUS_2 1
#define I2C_BAD_BUS_2 2
#define I2C_GOOD_HANDLE 0
#define I2C_BAD_HANDLE 1
#define I2C_GOOD_REGISTER 0x10
#define I2C_GOOD_REGISTER_2 0x20
#define I2C_BAD_REGISTER 0x30
#define FIFO_NOT_OVERFLOWED 0x2F
#define FIFO_OVERFLOWED 0x10

template<typename T>
bool hal_pigpioGpioSet(
  uint16_t gpio_id,
  std::shared_ptr<rclcpp::Client<T>> serviceClient,
  rclcpp::executors::SingleThreadedExecutor * executor)
{
  auto request = std::make_shared<typename T::Request>();

  request->gpio_id = gpio_id;

  auto future = serviceClient->async_send_request(request);

  executor->spin_until_future_complete(future);

  return future.get()->has_succeeded;
}

class PigioCheckerNode : public rclcpp::Node
{
  using HalPigpioSetInputMode_t = hal_pigpio_interfaces::srv::HalPigpioSetInputMode;
  using HalPigpioSetOutputMode_t = hal_pigpio_interfaces::srv::HalPigpioSetOutputMode;
  using HalPigpioGetMode_t = hal_pigpio_interfaces::srv::HalPigpioGetMode;
  using HalPigpioSetPullUp_t = hal_pigpio_interfaces::srv::HalPigpioSetPullUp;
  using HalPigpioSetPullDown_t = hal_pigpio_interfaces::srv::HalPigpioSetPullDown;
  using HalPigpioClearResistor_t = hal_pigpio_interfaces::srv::HalPigpioClearResistor;
  using HalPigpioSetPwmDutycycle_t = hal_pigpio_interfaces::srv::HalPigpioSetPwmDutycycle;
  using HalPigpioSetPwmFrequency_t = hal_pigpio_interfaces::srv::HalPigpioSetPwmFrequency;
  using HalPigpioSetGpioHigh_t = hal_pigpio_interfaces::srv::HalPigpioSetGpioHigh;
  using HalPigpioSetGpioLow_t = hal_pigpio_interfaces::srv::HalPigpioSetGpioLow;
  using HalPigpioSendTriggerPulse_t = hal_pigpio_interfaces::srv::HalPigpioSendTriggerPulse;
  using HalPigpioReadGpio_t = hal_pigpio_interfaces::srv::HalPigpioReadGpio;
  using HalPigpioSetCallback_t = hal_pigpio_interfaces::srv::HalPigpioSetCallback;
  using HalPigpioSetEncoderCallback_t = hal_pigpio_interfaces::srv::HalPigpioSetEncoderCallback;
  using HalPigpioSetMotorDirection_t = hal_pigpio_interfaces::srv::HalPigpioSetMotorDirection;
  using HalPigpioEdgeChangeMsg_t = hal_pigpio_interfaces::msg::HalPigpioEdgeChange;
  using HalPigpioEncoderCountMsg_t = hal_pigpio_interfaces::msg::HalPigpioEncoderCount;
  using HalPigpioI2cOpen_t = hal_pigpio_interfaces::srv::HalPigpioI2cOpen;
  using HalPigpioI2cClose_t = hal_pigpio_interfaces::srv::HalPigpioI2cClose;
  using HalPigpioI2cReadByteData_t = hal_pigpio_interfaces::srv::HalPigpioI2cReadByteData;
  using HalPigpioI2cReadWordData_t = hal_pigpio_interfaces::srv::HalPigpioI2cReadWordData;
  using HalPigpioI2cReadBlockData_t = hal_pigpio_interfaces::srv::HalPigpioI2cReadBlockData;
  using HalPigpioI2cWriteByteData_t = hal_pigpio_interfaces::srv::HalPigpioI2cWriteByteData;
  using HalPigpioI2cWriteWordData_t = hal_pigpio_interfaces::srv::HalPigpioI2cWriteWordData;
  using HalPigpioI2cWriteBlockData_t = hal_pigpio_interfaces::srv::HalPigpioI2cWriteBlockData;
  using HalPigpioI2cImuReading_t = hal_pigpio_interfaces::srv::HalPigpioI2cImuReading;
  using HalPigpioAnglesMsg_t = hal_pigpio_interfaces::msg::HalPigpioAngles;

public:
  PigioCheckerNode();
  ~PigioCheckerNode() = default;

  rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr changeStateClient;
  rclcpp::Client<HalPigpioSetInputMode_t>::SharedPtr setInputModeClient;
  rclcpp::Client<HalPigpioSetOutputMode_t>::SharedPtr setOutputModeClient;
  rclcpp::Client<HalPigpioGetMode_t>::SharedPtr getModeClient;
  rclcpp::Client<HalPigpioSetPullUp_t>::SharedPtr setPullUpClient;
  rclcpp::Client<HalPigpioSetPullDown_t>::SharedPtr setPullDownClient;
  rclcpp::Client<HalPigpioClearResistor_t>::SharedPtr clearResistorClient;
  rclcpp::Client<HalPigpioSetPwmDutycycle_t>::SharedPtr setPwmDutycycleClient;
  rclcpp::Client<HalPigpioSetPwmFrequency_t>::SharedPtr setPwmFrequencyClient;
  rclcpp::Client<HalPigpioSetGpioHigh_t>::SharedPtr setGpioHighClient;
  rclcpp::Client<HalPigpioSetGpioLow_t>::SharedPtr setGpioLowClient;
  rclcpp::Client<HalPigpioSendTriggerPulse_t>::SharedPtr sendTriggerPulseClient;
  rclcpp::Client<HalPigpioReadGpio_t>::SharedPtr readGpioClient;
  rclcpp::Client<HalPigpioSetCallback_t>::SharedPtr setCallbackClient;
  rclcpp::Client<HalPigpioSetEncoderCallback_t>::SharedPtr setEncoderCallbackClient;
  rclcpp::Client<HalPigpioSetMotorDirection_t>::SharedPtr setMotorDirectionClient;
  rclcpp::Subscription<HalPigpioEdgeChangeMsg_t>::SharedPtr pigpioEdgeChangeSub;
  rclcpp::Subscription<HalPigpioEncoderCountMsg_t>::SharedPtr pigpioEncoderCountSub;
  rclcpp::Client<HalPigpioI2cOpen_t>::SharedPtr i2cOpenClient;
  rclcpp::Client<HalPigpioI2cClose_t>::SharedPtr i2cCloseClient;
  rclcpp::Client<HalPigpioI2cReadByteData_t>::SharedPtr i2cReadByteDataClient;
  rclcpp::Client<HalPigpioI2cReadWordData_t>::SharedPtr i2cReadWordDataClient;
  rclcpp::Client<HalPigpioI2cReadBlockData_t>::SharedPtr i2cReadBlockDataClient;
  rclcpp::Client<HalPigpioI2cWriteByteData_t>::SharedPtr i2cWriteByteDataClient;
  rclcpp::Client<HalPigpioI2cWriteWordData_t>::SharedPtr i2cWriteWordDataClient;
  rclcpp::Client<HalPigpioI2cWriteBlockData_t>::SharedPtr i2cWriteBlockDataClient;
  rclcpp::Client<HalPigpioI2cImuReading_t>::SharedPtr i2cImuReadingClient;
  rclcpp::Subscription<HalPigpioAnglesMsg_t>::SharedPtr anglesSubscriber;

  uint8_t edgeChangeMsg_gpioId;
  uint8_t edgeChangeMsg_edgeChangeType;
  uint32_t edgeChangeMsg_timeSinceBoot_us;

  std::map<uint8_t, int32_t> motorsEC;

  std::promise<float> imuAnglePhi;
  std::promise<float> imuAngleTheta;
  std::promise<float> imuAnglePsi;

  void changePigpioNodeToState(std::uint8_t transition);
  bool setPwmDutycycle(
    uint16_t gpio_id,
    uint8_t dutycycle,
    rclcpp::executors::SingleThreadedExecutor * executor);
  bool setPwmFrequency(
    uint16_t gpio_id,
    uint16_t frequency,
    rclcpp::executors::SingleThreadedExecutor * executor);
  bool sendTriggerPulse(
    uint16_t gpio_id,
    uint16_t pulse_length_in_us,
    rclcpp::executors::SingleThreadedExecutor * executor);
  bool readGpioAndCheckLevel(
    uint16_t gpio_id,
    uint8_t gpio_level,
    rclcpp::executors::SingleThreadedExecutor * executor);
  bool setCallback(
    uint16_t gpio_id,
    uint8_t edge_change_type,
    rclcpp::executors::SingleThreadedExecutor * executor);
  bool setEncoderCallback(
    uint16_t gpio_id,
    uint8_t edge_change_type,
    uint8_t motor_id,
    rclcpp::executors::SingleThreadedExecutor * executor);
  bool setMotorDirection(
    bool is_direction_forward,
    uint8_t motor_id,
    rclcpp::executors::SingleThreadedExecutor * executor);
  void edgeChangeCallback(const HalPigpioEdgeChangeMsg_t & msg);
  void encoderCountCallback(const HalPigpioEncoderCountMsg_t & msg);
  int32_t i2cOpen(
    uint16_t bus,
    uint16_t address,
    rclcpp::executors::SingleThreadedExecutor * executor);
  bool i2cClose(
    int32_t handle,
    rclcpp::executors::SingleThreadedExecutor * executor);
  bool i2cWriteByteData(
    int32_t i2cHandle,
    uint8_t deviceRegister,
    uint8_t value,
    rclcpp::executors::SingleThreadedExecutor * executor);
  bool i2cWriteWordData(
    int32_t i2cHandle,
    uint8_t deviceRegister,
    uint16_t value,
    rclcpp::executors::SingleThreadedExecutor * executor);
  bool i2cWriteBlockData(
    int32_t i2cHandle,
    uint8_t deviceRegister,
    std::vector<uint8_t> dataBlock,
    uint8_t length,
    rclcpp::executors::SingleThreadedExecutor * executor);
  uint8_t i2cReadByteData(
    int32_t i2cHandle,
    uint8_t deviceRegister,
    rclcpp::executors::SingleThreadedExecutor * executor);
  uint16_t i2cReadWordData(
    int32_t i2cHandle,
    uint8_t deviceRegister,
    rclcpp::executors::SingleThreadedExecutor * executor);
  std::vector<uint8_t> i2cReadBlockData(
    int32_t i2cHandle,
    uint8_t deviceRegister,
    uint8_t length,
    rclcpp::executors::SingleThreadedExecutor * executor);
  void i2cStartImuReading(int32_t i2cHandle, rclcpp::executors::SingleThreadedExecutor * executor);
  void i2cStopImuReading(int32_t i2cHandle, rclcpp::executors::SingleThreadedExecutor * executor);
  void getAngles(const HalPigpioAnglesMsg_t & message);
};

/* Test fixture */
class PigpioTest : public testing::Test
{
protected:
  std::shared_ptr<Pigpio> pigpio;
  std::shared_ptr<PigioCheckerNode> pigpioChecker;
  rclcpp::executors::SingleThreadedExecutor executor;

  void SetUp()
  {
    pigpioChecker = std::make_shared<PigioCheckerNode>();
    pigpio = std::make_shared<Pigpio>();

    executor.add_node(pigpio->get_node_base_interface());
    executor.add_node(pigpioChecker);

    pigpioChecker->changePigpioNodeToState(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    executor.spin_some();
    pigpioChecker->changePigpioNodeToState(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
    executor.spin_some();
  }

  void TearDown()
  {
    pigpioChecker->changePigpioNodeToState(
      lifecycle_msgs::msg::Transition::TRANSITION_ACTIVE_SHUTDOWN);
    executor.spin_some();
    executor.cancel();
    executor.remove_node(pigpio->get_node_base_interface());
    executor.remove_node(pigpioChecker);
    pigpio.reset();
    pigpioChecker.reset();
  }
};

#endif  // HAL_PIGPIO_TESTS_HPP_
