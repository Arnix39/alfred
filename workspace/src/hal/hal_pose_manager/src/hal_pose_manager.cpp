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

#include "hal_pose_manager.hpp"

using namespace std::placeholders;

HalPoseManager::HalPoseManager()
: rclcpp_lifecycle::LifecycleNode{"hal_pose_manager_node"},
  encodersCount{.rightCurrrent = 0, .rightPrevious = 0,
    .leftCurrrent = 0, .leftPrevious = 0,
    .timestampNs = 0},
  wheelsVelocity{.right = 0, .left = 0}
{
}

LifecycleCallbackReturn_t HalPoseManager::on_configure(
  const rclcpp_lifecycle::State & previous_state)
{
  odometryPublisher = this->create_publisher<OdometryMsg_t>("odometry", 10);
  wheelsVelocityCmdPublisher = this->create_publisher<HalMotorControlCommandMsg_t>(
    "wheelsVelocityCmd", 10);
  twistSubscriber = this->create_subscription<TwistMsg_t>(
    "cmd_velocity", 10, std::bind(&HalPoseManager::wheelsVelocityCommand, this, _1));
  motorsECSubscriber = this->create_subscription<HalMotorControlEncodersMsg_t>(
    "motorsEncoderCountValue", 10,
    std::bind(&HalPoseManager::wheelsVelocityComputation, this, _1));

  RCLCPP_INFO(get_logger(), "Node configured!");

  return LifecycleCallbackReturn_t::SUCCESS;
}

LifecycleCallbackReturn_t HalPoseManager::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
  odometryPublisher->on_activate();
  wheelsVelocityCmdPublisher->on_activate();

  RCLCPP_INFO(get_logger(), "Node activated!");

  return LifecycleCallbackReturn_t::SUCCESS;
}

LifecycleCallbackReturn_t HalPoseManager::on_deactivate(
  const rclcpp_lifecycle::State & previous_state)
{
  odometryPublisher->on_deactivate();
  wheelsVelocityCmdPublisher->on_deactivate();

  RCLCPP_INFO(get_logger(), "Node deactivated!");

  return LifecycleCallbackReturn_t::SUCCESS;
}

LifecycleCallbackReturn_t HalPoseManager::on_cleanup(const rclcpp_lifecycle::State & previous_state)
{
  odometryPublisher.reset();
  wheelsVelocityCmdPublisher.reset();

  RCLCPP_INFO(get_logger(), "Node unconfigured!");

  return LifecycleCallbackReturn_t::SUCCESS;
}

LifecycleCallbackReturn_t HalPoseManager::on_shutdown(
  const rclcpp_lifecycle::State & previous_state)
{
  odometryPublisher.reset();
  wheelsVelocityCmdPublisher.reset();

  RCLCPP_INFO(get_logger(), "Node shutdown!");

  return LifecycleCallbackReturn_t::SUCCESS;
}

LifecycleCallbackReturn_t HalPoseManager::on_error(const rclcpp_lifecycle::State & previous_state)
{
  return LifecycleCallbackReturn_t::FAILURE;
}

void HalPoseManager::wheelsVelocityCommand(const TwistMsg_t & msg)
{
  auto wheelsVelocityCommandMsg = HalMotorControlCommandMsg_t();
  WheelsVelocity wheelsVelocityCommand;

  wheelsVelocityCmdPublisher->publish(wheelsVelocityCommandMsg);
}

void HalPoseManager::wheelsVelocityComputation(const HalMotorControlEncodersMsg_t & msg)
{
  auto header = HeaderMsg_t();
  auto odometry = OdometryMsg_t();
  auto pose = PoseMsg_t();
  auto twist = TwistMsg_t();

  header.stamp = rclcpp::Clock().now();
  header.frame_id = "Body";

  int32_t encoderCountDelta = 0;
  int32_t timeDelta = msg.header.stamp.nanosec - encodersCount.timestampNs;

  auto computeVelocity = [](int32_t encoderCountDelta, int32_t timeDelta) {
      return static_cast<int32_t>(round(
               static_cast<double>(encoderCountDelta) / timeDelta * MS_TO_NS));
    };

  encodersCount.leftPrevious = encodersCount.leftCurrrent;
  encodersCount.leftCurrrent = msg.motor_left_encoder_count;

  encoderCountDelta = encodersCount.leftCurrrent - encodersCount.leftPrevious;
  wheelsVelocity.left = computeVelocity(encoderCountDelta, timeDelta);

  encodersCount.rightPrevious = encodersCount.rightCurrrent;
  encodersCount.rightCurrrent = msg.motor_right_encoder_count;

  encoderCountDelta = encodersCount.rightCurrrent - encodersCount.rightPrevious;
  wheelsVelocity.right = computeVelocity(encoderCountDelta, timeDelta);

  odometry.header = std::move(header);
  odometry.child_frame_id = "Body";
  odometry.pose = std::move(pose);
  odometry.twist = std::move(twist);

  odometryPublisher->publish(odometry);
}
