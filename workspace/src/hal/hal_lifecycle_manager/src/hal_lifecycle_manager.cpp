#include "hal_lifecycle_manager.hpp"

HalLifecycleManager::HalLifecycleManager() :  rclcpp::Node("hal_lifecycle_manager_node"),
                                              hal_pigpio_node_get_state(this->create_client<lifecycle_msgs::srv::GetState>("hal_pigpio_node/get_state")),
                                              hal_proxsens_node_get_state(this->create_client<lifecycle_msgs::srv::GetState>("hal_proxsens_node/get_state")),
                                              hal_pigpio_node_change_state(this->create_client<lifecycle_msgs::srv::ChangeState>("hal_pigpio_node/change_state")),
                                              hal_proxsens_node_change_state(this->create_client<lifecycle_msgs::srv::ChangeState>("hal_proxsens_node/change_state"))
{
}