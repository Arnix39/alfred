#include "hal_pigpioInit.hpp"

using namespace std::placeholders;

PigpioInit::PigpioInit(std::shared_ptr<rclcpp::Node> node, int pigpioHandle) :  pigpioHandle(pigpioHandle),
                                                                                halPigpioNode(node),
                                                                                getHandleService(node->create_service<hal_pigpio_interfaces::srv::HalPigpioGetHandle>("hal_pigpioGetHandle", std::bind(&PigpioInit::getHandle, this, _1, _2))),
                                                                                getModeService(node->create_service<hal_pigpio_interfaces::srv::HalPigpioGetMode>("hal_pigpioGetMode", std::bind(&PigpioInit::getMode, this, _1, _2))),
                                                                                setInputModeService(node->create_service<hal_pigpio_interfaces::srv::HalPigpioSetInputMode>("hal_pigpioSetInputMode", std::bind(&PigpioInit::setInputMode, this, _1, _2))),
                                                                                setOutputModeService(node->create_service<hal_pigpio_interfaces::srv::HalPigpioSetOutputMode>("hal_pigpioSetOutputMode", std::bind(&PigpioInit::setOutputMode, this, _1, _2))),
                                                                                setPullUpService(node->create_service<hal_pigpio_interfaces::srv::HalPigpioSetPullUp>("hal_pigpioSetPullUp", std::bind(&PigpioInit::setPullUp, this, _1, _2))),
                                                                                setPullDownService(node->create_service<hal_pigpio_interfaces::srv::HalPigpioSetPullDown>("hal_pigpioSetPullDown", std::bind(&PigpioInit::setPullDown, this, _1, _2))),
                                                                                clearResistorService(node->create_service<hal_pigpio_interfaces::srv::HalPigpioClearResistor>("hal_pigpioClearResistor", std::bind(&PigpioInit::clearResistor, this, _1, _2))),
                                                                                heartbeatPublisher(node->create_publisher<hal_pigpio_interfaces::msg::HalPigpioHeartbeat>("hal_pigpioHeartbeat", 1000))
{
}

PigpioInit::~PigpioInit()
{
    RCLCPP_INFO(halPigpioNode->get_logger(),"Stopping pigpio daemon.");
    pigpio_stop(pigpioHandle);
}

void PigpioInit::getHandle(const std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioGetHandle::Request> request,
                           std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioGetHandle::Response> response)
                           
{
    (void)request;

    response->handle = pigpioHandle;
}

void PigpioInit::getMode(const std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioGetMode::Request> request,
                         std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioGetMode::Response> response)
{
    response->mode = get_mode(pigpioHandle, request->gpio_id);
    if (response->mode >= 0)
    {
        response->has_succeeded = true;
    }
    else
    {
        response->has_succeeded = false;
        RCLCPP_ERROR(halPigpioNode->get_logger(),"Failed to retrieve mode for GPIO %u!", request->gpio_id);
    }
}

void PigpioInit::setInputMode(const std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioSetInputMode::Request> request,
                              std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioSetInputMode::Response> response)
{
    if (set_mode(pigpioHandle, request->gpio_id, PI_INPUT) == 0)
    {
        response->has_succeeded = true;
        RCLCPP_INFO(halPigpioNode->get_logger(),"GPIO %u configured as input.", request->gpio_id);
    }
    else
    {
        response->has_succeeded = false;
        RCLCPP_ERROR(halPigpioNode->get_logger(),"Failed to configure GPIO %u as input!", request->gpio_id);
    }
}

void PigpioInit::setOutputMode(const std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioSetOutputMode::Request> request,
                               std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioSetOutputMode::Response> response)
{
    if (set_mode(pigpioHandle, request->gpio_id, PI_OUTPUT) == 0)
    {
        response->has_succeeded = true;
        RCLCPP_INFO(halPigpioNode->get_logger(),"GPIO %u configured as output.", request->gpio_id);
    }
    else
    {
        response->has_succeeded = false;
        RCLCPP_ERROR(halPigpioNode->get_logger(),"Failed to configure GPIO %u as output!", request->gpio_id);
    }
}

void PigpioInit::setPullUp(const std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioSetPullUp::Request> request,
                           std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioSetPullUp::Response> response)
{
    if (set_pull_up_down(pigpioHandle, request->gpio_id, PI_PUD_UP) == 0)
    {
        response->has_succeeded = true;
        RCLCPP_INFO(halPigpioNode->get_logger(),"Sucessfully set pull-up resistor for GPIO %u.", request->gpio_id);
    }
    else
    {
        response->has_succeeded = true;
        RCLCPP_INFO(halPigpioNode->get_logger(),"Failed to set pull-up resistor for GPIO %u!", request->gpio_id);
    }
}

void PigpioInit::setPullDown(const std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioSetPullDown::Request> request,
                             std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioSetPullDown::Response> response)
{
    if (set_pull_up_down(pigpioHandle, request->gpio_id, PI_PUD_DOWN) == 0)
    {
        response->has_succeeded = true;
        RCLCPP_INFO(halPigpioNode->get_logger(),"Sucessfully set pull-down resistor for GPIO %u.", request->gpio_id);
    }
    else
    {
        response->has_succeeded = true;
        RCLCPP_INFO(halPigpioNode->get_logger(),"Failed to set pull-down resistor for GPIO %u!", request->gpio_id);
    }
}

void PigpioInit::clearResistor(const std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioClearResistor::Request> request,
                               std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioClearResistor::Response> response)
{
    if (set_pull_up_down(pigpioHandle, request->gpio_id, PI_PUD_OFF) == 0)
    {
        response->has_succeeded = true;
        RCLCPP_INFO(halPigpioNode->get_logger(),"Sucessfully clear resistor for GPIO %u.", request->gpio_id);
    }
    else
    {
        response->has_succeeded = true;
        RCLCPP_INFO(halPigpioNode->get_logger(),"Failed to clear resistor for GPIO %u!", request->gpio_id);
    }
}

void PigpioInit::publishHeartbeat(void)
{
    auto heartbeat = hal_pigpio_interfaces::msg::HalPigpioHeartbeat();
    heartbeat.is_alive = true;
    heartbeatPublisher->publish(heartbeat);
}