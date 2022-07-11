#include "hal_proxsens.hpp"

using namespace std::placeholders;
using namespace std::chrono_literals;

Proxsens::Proxsens(std::shared_ptr<rclcpp::Node> node) : halProxsensNode(node),
                                                         edgeChangeType(NO_CHANGE),
                                                         timestamp(0),
                                                         echoCallbackId(0),
                                                         distanceInCm(UINT16_MAX),
                                                         pigpioNodeStarted(false),
                                                         isStarted(false),
                                                         gpioSetInputClient(node->create_client<hal_pigpio_interfaces::srv::HalPigpioSetInputMode>("hal_pigpioSetInputMode")),
                                                         gpioSetOutputClient(node->create_client<hal_pigpio_interfaces::srv::HalPigpioSetOutputMode>("hal_pigpioSetOutputMode")),
                                                         gpioSetCallbackClient(node->create_client<hal_pigpio_interfaces::srv::HalPigpioSetCallback>("hal_pigpioSetCallback")),
                                                         gpioSendTriggerPulseClient(node->create_client<hal_pigpio_interfaces::srv::HalPigpioSendTriggerPulse>("hal_pigpioSendTriggerPulse")),
                                                         gpioSetGpioHighClient(node->create_client<hal_pigpio_interfaces::srv::HalPigpioSetGpioHigh>("hal_pigpioSetGpioHigh")),
                                                         proxsensDistancePub(node->create_publisher<hal_proxsens_interfaces::msg::HalProxsens>("proxSensorValue", 1000)),
                                                         proxsensEdgeChangeSub(node->create_subscription<hal_pigpio_interfaces::msg::HalPigpioEdgeChange>("gpioEdgeChange", 1000, std::bind(&Proxsens::edgeChangeCallback, this, _1))),
                                                         proxsensPigpioHBSub(node->create_subscription<hal_pigpio_interfaces::msg::HalPigpioHeartbeat>("hal_pigpioHeartbeat", 1000, std::bind(&Proxsens::pigpioHeartbeatCallback, this, _1)))
{
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
    }
}

void Proxsens::pigpioHeartbeatCallback(const hal_pigpio_interfaces::msg::HalPigpioHeartbeat &msg)
{
    pigpioNodeStarted = msg.is_alive;
}

bool Proxsens::isPigpioNodeStarted(void)
{
    return pigpioNodeStarted;
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
    if (rclcpp::spin_until_future_complete(halProxsensNode, setInputModeResult) != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(halProxsensNode->get_logger(), "Failed to call service setInputMode");
    }

    setCallbackRequest->gpio_id = PROXSENS_ECHO_GPIO;
    setCallbackRequest->edge_change_type = AS_EITHER_EDGE;

    auto setCallbackResult = gpioSetCallbackClient->async_send_request(setCallbackRequest);
    if (rclcpp::spin_until_future_complete(halProxsensNode, setCallbackResult) == rclcpp::FutureReturnCode::SUCCESS)
    {
        if (setCallbackResult.get()->has_succeeded)
        {
            echoCallbackId = setCallbackResult.get()->callback_id;
        }
    }
    else
    {
        RCLCPP_ERROR(halProxsensNode->get_logger(), "Failed to call service setCallback");
    }

    setOutputModeForTriggerRequest->gpio_id = PROXSENS_TRIGGER_GPIO;
    auto setOutputModeForTriggerResult = gpioSetOutputClient->async_send_request(setOutputModeForTriggerRequest);
    if (rclcpp::spin_until_future_complete(halProxsensNode, setOutputModeForTriggerResult) != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(halProxsensNode->get_logger(), "Failed to call service setOutputMode for trigger");
    }

    setOutputModeForShifterRequest->gpio_id = PROXSENS_LEVEL_SHIFTER_OE_GPIO;
    auto setOutputModeForShifterResult = gpioSetOutputClient->async_send_request(setOutputModeForShifterRequest);
    if (rclcpp::spin_until_future_complete(halProxsensNode, setOutputModeForShifterResult) != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(halProxsensNode->get_logger(), "Failed to call service setOutputMode for shifter");
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

void Proxsens::starts(void)
{
    isStarted = true;
}

bool Proxsens::isNotStarted(void)
{
    return !isStarted;
}