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

#include "package_template.hpp"

PackageTemplate::PackageTemplate()
: rclcpp_lifecycle::LifecycleNode{"package_template_node"}
{
}

LifecycleCallbackReturn_t PackageTemplate::on_configure(
  const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(get_logger(), "package_template node configured!");

  return LifecycleCallbackReturn_t::SUCCESS;
}

LifecycleCallbackReturn_t PackageTemplate::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(get_logger(), "package_template node activated!");

  return LifecycleCallbackReturn_t::SUCCESS;
}

LifecycleCallbackReturn_t PackageTemplate::on_deactivate(
  const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(get_logger(), "package_template node deactivated!");

  return LifecycleCallbackReturn_t::SUCCESS;
}

LifecycleCallbackReturn_t PackageTemplate::on_cleanup(
  const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(get_logger(), "package_template node unconfigured!");

  return LifecycleCallbackReturn_t::SUCCESS;
}

LifecycleCallbackReturn_t PackageTemplate::on_shutdown(
  const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(get_logger(), "package_template node shutdown!");

  return LifecycleCallbackReturn_t::SUCCESS;
}

LifecycleCallbackReturn_t PackageTemplate::on_error(
  const rclcpp_lifecycle::State & previous_state)
{
  return LifecycleCallbackReturn_t::FAILURE;
}
