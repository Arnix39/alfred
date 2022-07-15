#include "hal_proxsens.hpp"

using namespace std::placeholders;
using namespace std::chrono_literals;

Proxsens::Proxsens() : rclcpp_lifecycle::LifecycleNode("hal_proxsens_node"),
                       edgeChangeType(NO_CHANGE),
                       timestamp(0),
                       echoCallbackId(0),
                       distanceInCm(UINT16_MAX)
{
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Proxsens::on_configure(const rclcpp_lifecycle::State &)
{
    gpioSetInputClient = this->create_client<hal_pigpio_interfaces::srv::HalPigpioSetInputMode>("hal_pigpioSetInputMode");
    gpioSetOutputClient = this->create_client<hal_pigpio_interfaces::srv::HalPigpioSetOutputMode>("hal_pigpioSetOutputMode");
    gpioSetCallbackClient = this->create_client<hal_pigpio_interfaces::srv::HalPigpioSetCallback>("hal_pigpioSetCallback");
    gpioSendTriggerPulseClient = this->create_client<hal_pigpio_interfaces::srv::HalPigpioSendTriggerPulse>("hal_pigpioSendTriggerPulse");
    gpioSetGpioHighClient = this->create_client<hal_pigpio_interfaces::srv::HalPigpioSetGpioHigh>("hal_pigpioSetGpioHigh");

    proxsensDistancePub = this->create_publisher<hal_proxsens_interfaces::msg::HalProxsens>("proxSensorValue", 1000);

    proxsensEdgeChangeSub = this->create_subscription<hal_pigpio_interfaces::msg::HalPigpioEdgeChange>("gpioEdgeChange", 1000, std::bind(&Proxsens::edgeChangeCallback, this, _1));
    
    RCLCPP_INFO(get_logger(),"proxsens node configured!");

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Proxsens::on_activate(const rclcpp_lifecycle::State &)
{
    proxsensDistancePub->on_activate();

    configureGpios();
    enableOutputLevelShifter();
    
    RCLCPP_INFO(get_logger(),"proxsens node activated!");

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Proxsens::on_deactivate(const rclcpp_lifecycle::State &)
{
    proxsensDistancePub->on_deactivate();

    edgeChangeType = NO_CHANGE;
    timestamp = 0;
    echoCallbackId = 0;
    distanceInCm = UINT16_MAX;

    RCLCPP_INFO(get_logger(),"proxsens node deactivated!");

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Proxsens::on_cleanup(const rclcpp_lifecycle::State &)
{
    proxsensDistancePub.reset();

    RCLCPP_INFO(get_logger(),"proxsens node unconfigured!");

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Proxsens::on_shutdown(const rclcpp_lifecycle::State &)
{
    proxsensDistancePub.reset();

    RCLCPP_INFO(get_logger(),"proxsens node shutdown!");

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void Proxsens::edgeChangeCallback(const hal_pigpio_interfaces::msg::HalPigpioEdgeChange &msg)
{
    static uint8_t lastEdgeChangeType = NO_CHANGE;
    static uint32_t lastTimestamp = 0;

    uint32_t edgeLength = 0;

    if (msg.gpio_id == PROXSENS_ECHO_GPIO)
    {
        edgeChangeType = msg.edge_change_type;
        timestamp = msg.time_since_boot_us;

        if ((edgeChangeType == FALLING_EDGE) && (lastEdgeChangeType == RISING_EDGE))
        {
            if (timestamp < lastTimestamp)
            {
                edgeLength = UINT32_MAX - lastTimestamp + timestamp;
            }
            else
            {
                edgeLength = timestamp - lastTimestamp;
            }

            distanceInCm = static_cast<uint16_t>(edgeLength / 58.0);
        }
        else if (edgeChangeType == RISING_EDGE)
        {
            lastEdgeChangeType = edgeChangeType;
            lastTimestamp = timestamp;
        }
        else
        {
            lastEdgeChangeType = FALLING_EDGE;
            lastTimestamp = timestamp;
        }
    }
}

void Proxsens::publishDistance(void)
{
    auto distance = hal_proxsens_interfaces::msg::HalProxsens();

    distance.distance_in_cm = distanceInCm;
    proxsensDistancePub->publish(distance);
}

void Proxsens::configureGpios(void)
{
    auto setInputModeRequest = std::make_shared<hal_pigpio_interfaces::srv::HalPigpioSetInputMode::Request>();
    auto setOutputModeForTriggerRequest = std::make_shared<hal_pigpio_interfaces::srv::HalPigpioSetOutputMode::Request>();
    auto setOutputModeForShifterRequest = std::make_shared<hal_pigpio_interfaces::srv::HalPigpioSetOutputMode::Request>();
    auto setCallbackRequest = std::make_shared<hal_pigpio_interfaces::srv::HalPigpioSetCallback::Request>();

    setInputModeRequest->gpio_id = PROXSENS_ECHO_GPIO;
    auto setInputModeResult = gpioSetInputClient->async_send_request(setInputModeRequest);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), setInputModeResult) != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(get_logger(), "Failed to call service setInputMode");
    }

    setCallbackRequest->gpio_id = PROXSENS_ECHO_GPIO;
    setCallbackRequest->edge_change_type = AS_EITHER_EDGE;

    auto setCallbackResult = gpioSetCallbackClient->async_send_request(setCallbackRequest);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), setCallbackResult) == rclcpp::FutureReturnCode::SUCCESS)
    {
        if (setCallbackResult.get()->has_succeeded)
        {
            echoCallbackId = setCallbackResult.get()->callback_id;
        }
    }
    else
    {
        RCLCPP_ERROR(get_logger(), "Failed to call service setCallback");
    }

    setOutputModeForTriggerRequest->gpio_id = PROXSENS_TRIGGER_GPIO;
    auto setOutputModeForTriggerResult = gpioSetOutputClient->async_send_request(setOutputModeForTriggerRequest);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), setOutputModeForTriggerResult) != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(get_logger(), "Failed to call service setOutputMode for trigger");
    }

    setOutputModeForShifterRequest->gpio_id = PROXSENS_LEVEL_SHIFTER_OE_GPIO;
    auto setOutputModeForShifterResult = gpioSetOutputClient->async_send_request(setOutputModeForShifterRequest);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), setOutputModeForShifterResult) != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(get_logger(), "Failed to call service setOutputMode for shifter");
    }
}

void Proxsens::trigger(void)
{
    auto sendTriggerPulseRequest = std::make_shared<hal_pigpio_interfaces::srv::HalPigpioSendTriggerPulse::Request>();

    sendTriggerPulseRequest->gpio_id = PROXSENS_TRIGGER_GPIO;
    sendTriggerPulseRequest->pulse_length_in_us = PROXSENS_TRIGGER_LENGTH_US;

    auto sendTriggerPulseResult = gpioSendTriggerPulseClient->async_send_request(sendTriggerPulseRequest);
}

void Proxsens::enableOutputLevelShifter(void)
{
    auto setGpioHighRequest = std::make_shared<hal_pigpio_interfaces::srv::HalPigpioSetGpioHigh::Request>();

    setGpioHighRequest->gpio_id = PROXSENS_LEVEL_SHIFTER_OE_GPIO;
    auto setGpioHighResult = gpioSetGpioHighClient->async_send_request(setGpioHighRequest);
}

void Proxsens::publishAndGetDistance(void)
{
    publishDistance();
    trigger();
}