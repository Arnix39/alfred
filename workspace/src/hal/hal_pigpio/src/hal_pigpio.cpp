#include "hal_pigpio.hpp"

using namespace std::chrono_literals;

Pigpio::Pigpio() : rclcpp_lifecycle::LifecycleNode("hal_pigpio_node"),
                   pigpioHandle(-1)
{
}

Pigpio::~Pigpio()
{
    for (uint callbackId : callbackList)
    {
        callback_cancel(callbackId);
    }
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Pigpio::on_configure(const rclcpp_lifecycle::State &)
{
    pigpioHandle = pigpio_start(NULL, NULL);
    if (pigpioHandle < 0)
    {
        RCLCPP_ERROR(get_logger(),"Pigpio daemon not running!");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }

    RCLCPP_INFO(get_logger(),"Pigpio handle: %d.", pigpioHandle);

    encoderCountTimer = create_wall_timer(5ms, std::bind(&Pigpio::publishEncoderCount, this));

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Pigpio::on_activate(const rclcpp_lifecycle::State &)
{
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Pigpio::on_deactivate(const rclcpp_lifecycle::State &)
{
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Pigpio::on_cleanup(const rclcpp_lifecycle::State &)
{
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Pigpio::on_shutdown(const rclcpp_lifecycle::State &)
{
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}