#include "hal_lifecycle_manager.hpp"

HalLifecycleManager::HalLifecycleManager() :  rclcpp::Node("hal_lifecycle_manager_node"),
                                              hal_pigpio_node_get_state(this->create_client<lifecycle_msgs::srv::GetState>("hal_pigpio_node/get_state")),
                                              hal_proxsens_node_get_state(this->create_client<lifecycle_msgs::srv::GetState>("hal_proxsens_node/get_state")),
                                              hal_pigpio_node_change_state(this->create_client<lifecycle_msgs::srv::ChangeState>("hal_pigpio_node/change_state")),
                                              hal_proxsens_node_change_state(this->create_client<lifecycle_msgs::srv::ChangeState>("hal_proxsens_node/change_state"))
{
}

void HalLifecycleManager::ActivateNodes(void)
{
    std::chrono::milliseconds timeout(100);
    auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();

    request->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE;

    auto future_result = hal_pigpio_node_change_state->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_result, timeout) != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(get_logger(), "Failed to transition hal_pigpio to inactive state.");
    }

    future_result = hal_proxsens_node_change_state->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_result, timeout) != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(get_logger(), "Failed to transition hal_proxsens to inactive state.");
    }

    request->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE;

    future_result = hal_pigpio_node_change_state->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_result, timeout) != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(get_logger(), "Failed to transition hal_pigpio to active state.");
    }

    future_result = hal_proxsens_node_change_state->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_result, timeout) != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(get_logger(), "Failed to transition hal_proxsens to active state.");
    }
}

void HalLifecycleManager::DeactivateNodes(void)
{
    std::chrono::milliseconds timeout(100);
    auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();

    request->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE;

    auto future_result = hal_pigpio_node_change_state->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_result, timeout) != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(get_logger(), "Failed to transition hal_pigpio to inactive state.");
    }

    future_result = hal_proxsens_node_change_state->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_result, timeout) != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(get_logger(), "Failed to transition hal_proxsens to inactive state.");
    }
}

void HalLifecycleManager::ShutdownNodes(void)
{
    std::chrono::milliseconds timeout(100);
    auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();

    request->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP;

    auto future_result = hal_pigpio_node_change_state->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_result, timeout) != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(get_logger(), "Failed to transition hal_pigpio to unconfigured state.");
    }

    future_result = hal_proxsens_node_change_state->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_result, timeout) != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(get_logger(), "Failed to transition hal_proxsens to unconfigured state.");
    }

    request->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_UNCONFIGURED_SHUTDOWN;

    future_result = hal_pigpio_node_change_state->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_result, timeout) != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(get_logger(), "Failed to transition hal_pigpio to shutdown state.");
    }

    future_result = hal_proxsens_node_change_state->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_result, timeout) != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(get_logger(), "Failed to transition hal_proxsens to shutdown state.");
    }
}