#ifndef COMMON_HPP_
#define COMMON_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

using LifecycleCallbackReturn_t =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

#endif  // COMMON_HPP_
