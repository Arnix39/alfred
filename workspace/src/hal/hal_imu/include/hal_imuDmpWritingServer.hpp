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

#ifndef HAL_IMUDMPWRITINGSERVER_HPP_
#define HAL_IMUDMPWRITINGSERVER_HPP_

#include <thread>
#include <vector>
#include <memory>

#include "rclcpp_action/rclcpp_action.hpp"

#include "hal_imuDmpMemory.hpp"
#include "common.hpp"
#include "mpu6050.hpp"
#include "hal_i2cRegistersServices.hpp"

// Services and messages headers (generated)
#include "hal_pigpio_interfaces/srv/hal_pigpio_i2c_write_byte_data.hpp"
#include "hal_pigpio_interfaces/srv/hal_pigpio_i2c_write_block_data.hpp"
#include "hal_imu_interfaces/action/hal_imu_write_dmp.hpp"
#include "hal_imu_interfaces/srv/hal_imu_get_handle.hpp"

using HalImuWriteDmpAction = hal_imu_interfaces::action::HalImuWriteDmp;
using HalImuWriteDmpGoal = rclcpp_action::ServerGoalHandle<HalImuWriteDmpAction>;

class ImuDmpWritingServer : public rclcpp_lifecycle::LifecycleNode
{
private:
  int32_t imuHandle;

  i2cWriteByteDataSyncClientNode_t i2cWriteByteDataSyncClient;
  i2cWriteBlockDataSyncClientNode_t i2cWriteBlockDataSyncClient;
  imuGetHandleSyncClientNode_t imuGetHandleSyncClient;

  rclcpp_action::Server<HalImuWriteDmpAction>::SharedPtr imuDmpWritingServer;
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const HalImuWriteDmpAction::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<HalImuWriteDmpGoal> goal_handle);
  void handle_accepted(const std::shared_ptr<HalImuWriteDmpGoal> goal_handle);

public:
  ImuDmpWritingServer();
  ~ImuDmpWritingServer() = default;

  LifecycleCallbackReturn_t on_configure(const rclcpp_lifecycle::State & previous_state);
  LifecycleCallbackReturn_t on_activate(const rclcpp_lifecycle::State & previous_state);
  LifecycleCallbackReturn_t on_deactivate(const rclcpp_lifecycle::State & previous_state);
  LifecycleCallbackReturn_t on_cleanup(const rclcpp_lifecycle::State & previous_state);
  LifecycleCallbackReturn_t on_shutdown(const rclcpp_lifecycle::State & previous_state);
  LifecycleCallbackReturn_t on_error(const rclcpp_lifecycle::State & previous_state);

  void writeDmp(const std::shared_ptr<HalImuWriteDmpGoal> goal_handle);
  bool writeData(uint8_t bank, uint8_t addressInBank, std::vector<uint8_t> data);
};

#endif  // HAL_IMUDMPWRITINGSERVER_HPP_
