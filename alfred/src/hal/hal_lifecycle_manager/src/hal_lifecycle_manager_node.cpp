// Copyright (c) 2023 Arnix Robotix
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

#include "hal_lifecycle_manager.hpp"
#include "hal_camera.hpp"
#include "hal_imuI2cInit.hpp"
#include "hal_imuDmpWritingServer.hpp"
#include "hal_imu.hpp"
#include "hal_motor_control.hpp"
#include "hal_pigpio.hpp"
#include "hal_pose_manager.hpp"
#include "hal_proxsens.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto lifecycleManagerNode = std::make_shared<hal::lifecycle_manager::LifecycleManager>();
  // auto pigpioNode = std::make_shared<hal::pigpio::Pigpio>();
  // auto proxsensNode = std::make_shared<hal::proxsens::Proxsens>();
  // auto motorControlNode = std::make_shared<hal::motor::control::MotorControl>();
  // auto imuI2cInitNode = std::make_shared<hal::imu::i2cInit::ImuI2cInit>();
  // auto dmpWritingServerNode = std::make_shared<hal::imu::dmp::DmpWritingServer>();
  // auto imuNode = std::make_shared<hal::imu::Imu>();
  // auto halPoseManagerNode = std::make_shared<hal::pose_manager::HalPoseManager>();
  // auto cameraNode = std::make_shared<hal::camera::Camera>();

  executor.add_node(lifecycleManagerNode->get_node_base_interface());
  // executor.add_node(pigpioNode->get_node_base_interface());
  // executor.add_node(proxsensNode->get_node_base_interface());
  // executor.add_node(motorControlNode->get_node_base_interface());
  // executor.add_node(imuI2cInitNode->get_node_base_interface());
  // executor.add_node(dmpWritingServerNode->get_node_base_interface());
  // executor.add_node(imuNode->get_node_base_interface());
  // executor.add_node(halPoseManagerNode->get_node_base_interface());
  // executor.add_node(cameraNode->get_node_base_interface());

  executor.spin();

  rclcpp::shutdown();
  return 0;
}
