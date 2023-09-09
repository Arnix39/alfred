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

#ifndef HAL_IMU_HPP_
#define HAL_IMU_HPP_

#include <vector>
#include <memory>
/* TODO(Arnix) To be moved with computeAngularVelAndLinearAcc()
#define _USE_MATH_DEFINES
#include <cmath>
*/

#include "rclcpp_action/rclcpp_action.hpp"

#include "common.hpp"
#include "mpu6050.hpp"
#include "hal_i2cRegistersServices.hpp"

// Services and messages headers (generated)
#include "hal_pigpio_interfaces/srv/hal_pigpio_i2c_read_byte_data.hpp"
#include "hal_pigpio_interfaces/srv/hal_pigpio_i2c_write_byte_data.hpp"
#include "hal_pigpio_interfaces/srv/hal_pigpio_i2c_write_block_data.hpp"
#include "hal_pigpio_interfaces/srv/hal_pigpio_i2c_imu_reading.hpp"
#include "hal_imu_interfaces/action/hal_imu_write_dmp.hpp"
#include "hal_imu_interfaces/srv/hal_imu_get_handle.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace hal
{
namespace imu
{

#define GYROSCOPE_X_OFFSET  0
#define GYROSCOPE_Y_OFFSET  -34
#define GYROSCOPE_Z_OFFSET  -9

#define ACCELEROMETER_X_OFFSET -556
#define ACCELEROMETER_Y_OFFSET -1188
#define ACCELEROMETER_Z_OFFSET 873

struct SensorBias
{
  const char axis;
  const int16_t bias;
  const uint8_t msbRegister;
  const uint8_t lsbRegister;
};

using HalPigpioI2cImuReading_t = hal_pigpio_interfaces::srv::HalPigpioI2cImuReading;

using HalImuWriteDmpAction = hal_imu_interfaces::action::HalImuWriteDmp;
using HalImuWriteDmpGoal = rclcpp_action::ClientGoalHandle<HalImuWriteDmpAction>;

using ImuDataMsg_t = sensor_msgs::msg::Imu;

class Imu : public rclcpp_lifecycle::LifecycleNode
{
private:
  int32_t imuHandle;

  rclcpp::Client<HalPigpioI2cImuReading_t>::SharedPtr i2cImuReadingClient;
  i2cReadByteDataSyncClientNode_t i2cReadByteDataSyncClient;
  i2cWriteByteDataSyncClientNode_t i2cWriteByteDataSyncClient;
  i2cWriteBlockDataSyncClientNode_t i2cWriteBlockDataSyncClient;
  imuGetHandleSyncClientNode_t imuGetHandleSyncClient;

  rclcpp_action::Client<HalImuWriteDmpAction>::SharedPtr imuDmpWritingClient;

  void goal_response_callback(HalImuWriteDmpGoal::SharedPtr goal_handle);
  void feedback_callback(
    HalImuWriteDmpGoal::SharedPtr,
    const std::shared_ptr<const HalImuWriteDmpAction::Feedback> feedback);
  void result_callback(const HalImuWriteDmpGoal::WrappedResult & result);

  rclcpp::Subscription<ImuDataMsg_t>::SharedPtr imuSubscriber;
  rclcpp_lifecycle::LifecyclePublisher<ImuDataMsg_t>::SharedPtr imuDataPublisher;

public:
  Imu();
  ~Imu() = default;

  LifecycleCallbackReturn_t on_configure(const rclcpp_lifecycle::State & previous_state);
  LifecycleCallbackReturn_t on_activate(const rclcpp_lifecycle::State & previous_state);
  LifecycleCallbackReturn_t on_deactivate(const rclcpp_lifecycle::State & previous_state);
  LifecycleCallbackReturn_t on_cleanup(const rclcpp_lifecycle::State & previous_state);
  LifecycleCallbackReturn_t on_shutdown(const rclcpp_lifecycle::State & previous_state);
  LifecycleCallbackReturn_t on_error(const rclcpp_lifecycle::State & previous_state);

  void dmpInit(void);
  void setDmpRate(int32_t imuHandle, uint16_t rate);
  void setMpuRate(int32_t imuHandle, uint16_t rate);
  void enableDmpAndStartReading(int32_t imuHandle);
  void configureDmpFeatures(int32_t imuHandle);
  void resetImu(int32_t imuHandle);
  void resetFifo(int32_t imuHandle);
  void setClockSource(int32_t imuHandle);
  bool writeDataToDmp(
    int32_t imuHandle, uint8_t bank, uint8_t addressInBank, std::vector<uint8_t> data);
  void startImuReading(void);
  void stopImuReading(void);
  void setConfiguration(int32_t imuHandle);
  void setGyroscopeSensitivity(int32_t imuHandle);
  void setAccelerometerSensitivity(int32_t imuHandle);
  void setAccelerometerOffsets(int32_t imuHandle);
  void setGyroscopeOffsets(int32_t imuHandle);
  bool writeSensorBiases(int32_t imuHandle, const std::vector<SensorBias> sensorBiases);

  void imuForwarder(const ImuDataMsg_t & msg);
};

}  // namespace imu
}  // namespace hal

#endif  // HAL_IMU_HPP_
