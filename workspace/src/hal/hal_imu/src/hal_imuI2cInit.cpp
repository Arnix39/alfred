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

#include "hal_imuI2cInit.hpp"

using namespace std::placeholders;
using namespace std::chrono_literals;

ImuI2cInit::ImuI2cInit()
: rclcpp_lifecycle::LifecycleNode{"hal_imuI2cInit_node"},
  imuHandle{MPU6050_I2C_NO_HANDLE}
{
}

LifecycleCallbackReturn_t ImuI2cInit::on_configure(const rclcpp_lifecycle::State & previous_state)
{
  i2cOpenClient = this->create_client<HalPigpioI2cOpen_t>("hal_pigpioI2cOpen");
  i2cCloseClient = this->create_client<HalPigpioI2cClose_t>("hal_pigpioI2cClose");

  getHandleService = this->create_service<HalImuGetHandle_t>(
    "hal_imuGetHandle", std::bind(&ImuI2cInit::getHandle, this, _1, _2));

  RCLCPP_INFO(get_logger(), "Node configured!");

  return LifecycleCallbackReturn_t::SUCCESS;
}

LifecycleCallbackReturn_t ImuI2cInit::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  initI2cCommunication();

  RCLCPP_INFO(get_logger(), "Node activated!");

  return LifecycleCallbackReturn_t::SUCCESS;
}

LifecycleCallbackReturn_t ImuI2cInit::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  auto i2cCloseRequest = std::make_shared<HalPigpioI2cClose_t::Request>();

  i2cCloseRequest->handle = imuHandle;

  auto i2cCloseCallback = [this](I2cCloseFuture_t future)
    {
      if (!future.get()->has_succeeded) {
        RCLCPP_ERROR(get_logger(), "Error while closing I2C communication with IMU");
      }
    };
  auto i2cCloseFuture = i2cCloseClient->async_send_request(i2cCloseRequest, i2cCloseCallback);

  imuHandle = MPU6050_I2C_NO_HANDLE;

  RCLCPP_INFO(get_logger(), "Node deactivated!");

  return LifecycleCallbackReturn_t::SUCCESS;
}

LifecycleCallbackReturn_t ImuI2cInit::on_cleanup(const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(get_logger(), "Node unconfigured!");

  return LifecycleCallbackReturn_t::SUCCESS;
}

LifecycleCallbackReturn_t ImuI2cInit::on_shutdown(const rclcpp_lifecycle::State & previous_state)
{
  if (previous_state.label().c_str() == "Active") {
    auto i2cCloseRequest = std::make_shared<HalPigpioI2cClose_t::Request>();

    i2cCloseRequest->handle = imuHandle;

    auto i2cCloseCallback = [this](I2cCloseFuture_t future)
      {
        if (!future.get()->has_succeeded) {
          RCLCPP_ERROR(get_logger(), "Error while closing I2C communication with IMU");
        }
      };
    auto i2cCloseFuture = i2cCloseClient->async_send_request(i2cCloseRequest, i2cCloseCallback);

    imuHandle = MPU6050_I2C_NO_HANDLE;
  }

  RCLCPP_INFO(get_logger(), "Node shutdown!");

  return LifecycleCallbackReturn_t::SUCCESS;
}

LifecycleCallbackReturn_t ImuI2cInit::on_error(const rclcpp_lifecycle::State & previous_state)
{
  return LifecycleCallbackReturn_t::FAILURE;
}

void ImuI2cInit::getHandle(
  const std::shared_ptr<HalImuGetHandle_t::Request> request,
  std::shared_ptr<HalImuGetHandle_t::Response> response)
{
  (void)request;

  response->handle = imuHandle;
}

void ImuI2cInit::initI2cCommunication(void)
{
  auto i2cOpenRequest = std::make_shared<HalPigpioI2cOpen_t::Request>();

  i2cOpenRequest->bus = IMU_I2C_BUS;
  i2cOpenRequest->address = MPU6050_I2C_ADDRESS;

  auto i2cOpenCallback = [this](I2cOpenFuture_t future)
    {
      if (future.get()->has_succeeded && future.get()->handle >= 0) {
        this->imuHandle = future.get()->handle;
        RCLCPP_INFO(get_logger(), "IMU I2C handle %u received.", this->imuHandle);
      } else {
        RCLCPP_ERROR(get_logger(), "Unable to receive IMU I2C handle!");
      }
    };
  auto i2cOpenFuture = i2cOpenClient->async_send_request(i2cOpenRequest, i2cOpenCallback);
}
