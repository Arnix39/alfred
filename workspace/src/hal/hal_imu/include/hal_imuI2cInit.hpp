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

#ifndef HAL_IMUI2CINIT_HPP_
#define HAL_IMUI2CINIT_HPP_

#include <memory>

#include "common.hpp"
#include "mpu6050.hpp"

// Services and messages headers (generated)
#include "hal_pigpio_interfaces/srv/hal_pigpio_i2c_open.hpp"
#include "hal_pigpio_interfaces/srv/hal_pigpio_i2c_close.hpp"
#include "hal_imu_interfaces/srv/hal_imu_get_handle.hpp"

#define IMU_I2C_BUS 0x1

using HalPigpioI2cOpen_t = hal_pigpio_interfaces::srv::HalPigpioI2cOpen;
using HalPigpioI2cClose_t = hal_pigpio_interfaces::srv::HalPigpioI2cClose;
using HalImuGetHandle_t = hal_imu_interfaces::srv::HalImuGetHandle;

using I2cOpenFuture_t = rclcpp::Client<HalPigpioI2cOpen_t>::SharedFuture;
using I2cCloseFuture_t = rclcpp::Client<HalPigpioI2cClose_t>::SharedFuture;

class ImuI2cInit : public rclcpp_lifecycle::LifecycleNode
{
private:
  int32_t imuHandle;

  rclcpp::Service<HalImuGetHandle_t>::SharedPtr getHandleService;
  rclcpp::Client<HalPigpioI2cOpen_t>::SharedPtr i2cOpenClient;
  rclcpp::Client<HalPigpioI2cClose_t>::SharedPtr i2cCloseClient;

public:
  ImuI2cInit();
  ~ImuI2cInit() = default;

  LifecycleCallbackReturn_t on_configure(const rclcpp_lifecycle::State & previous_state);
  LifecycleCallbackReturn_t on_activate(const rclcpp_lifecycle::State & previous_state);
  LifecycleCallbackReturn_t on_deactivate(const rclcpp_lifecycle::State & previous_state);
  LifecycleCallbackReturn_t on_cleanup(const rclcpp_lifecycle::State & previous_state);
  LifecycleCallbackReturn_t on_shutdown(const rclcpp_lifecycle::State & previous_state);
  LifecycleCallbackReturn_t on_error(const rclcpp_lifecycle::State & previous_state);

  void getHandle(
    const std::shared_ptr<HalImuGetHandle_t::Request> request,
    std::shared_ptr<HalImuGetHandle_t::Response> response);
  void initI2cCommunication(void);
};

#endif  // HAL_IMUI2CINIT_HPP_
