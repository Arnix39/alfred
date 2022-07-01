#include "hal_pigpioInit.hpp"

PigpioInit::PigpioInit(std::shared_ptr<rclcpp::Node> node, int pigpioHandle) :  pigpioHandle(pigpioHandle),
                                                                                halPigpioNode(node),
                                                                                getHandleService(node->advertiseService("hal_pigpioGetHandle", &PigpioInit::getHandle, this)),
                                                                                getModeService(node->advertiseService("hal_pigpioGetMode", &PigpioInit::getMode, this)),
                                                                                setInputModeService(node->advertiseService("hal_pigpioSetInputMode", &PigpioInit::setInputMode, this)),
                                                                                setOutputModeService(node->advertiseService("hal_pigpioSetOutputMode", &PigpioInit::setOutputMode, this)),
                                                                                setPullUpService(node->advertiseService("hal_pigpioSetPullUp", &PigpioInit::setPullUp, this)),
                                                                                setPullDownService(node->advertiseService("hal_pigpioSetPullDown", &PigpioInit::setPullDown, this)),
                                                                                clearResistorService(node->advertiseService("hal_pigpioClearResistor", &PigpioInit::clearResistor, this)),
                                                                                heartbeatPublisher(node->advertise<hal_pigpio::hal_pigpioHeartbeatMsg>("hal_pigpioHeartbeat", 1000))
{
}

PigpioInit::~PigpioInit()
{
    RCLCPP_INFO(halPigpioNode->get_logger(),"Stopping pigpio daemon.");
    pigpio_stop(pigpioHandle);
}

void PigpioInit::getHandle(hal_pigpio::hal_pigpioGetHandle::Request request,
                           hal_pigpio::hal_pigpioGetHandle::Response response)
{
    response->handle = pigpioHandle;
    return true;
}

void PigpioInit::getMode(hal_pigpio::hal_pigpioGetMode::Request request,
                         hal_pigpio::hal_pigpioGetMode::Response response)
{
    response->mode = get_mode(pigpioHandle, request->gpioId);
    if (response->mode >= 0)
    {
        response->has_succeeded = true;
    }
    else
    {
        response->has_succeeded = false;
        RCLCPP_ERROR(halPigpioNode->get_logger(),"Failed to retrieve mode for GPIO %u!", request->gpioId);
    }
    return true;
}

void PigpioInit::setInputMode(hal_pigpio::hal_pigpioSetInputMode::Request request,
                              hal_pigpio::hal_pigpioSetInputMode::Response response)
{
    if (set_mode(pigpioHandle, request->gpioId, PI_INPUT) == 0)
    {
        response->has_succeeded = true;
        RCLCPP_INFO(halPigpioNode->get_logger(),"GPIO %u configured as input.", request->gpioId);
    }
    else
    {
        response->has_succeeded = false;
        RCLCPP_ERROR(halPigpioNode->get_logger(),"Failed to configure GPIO %u as input!", request->gpioId);
    }

    return true;
}

void PigpioInit::setOutputMode(hal_pigpio::hal_pigpioSetOutputMode::Request request,
                               hal_pigpio::hal_pigpioSetOutputMode::Response response)
{
    if (set_mode(pigpioHandle, request->gpioId, PI_OUTPUT) == 0)
    {
        response->has_succeeded = true;
        RCLCPP_INFO(halPigpioNode->get_logger(),"GPIO %u configured as output.", request->gpioId);
    }
    else
    {
        response->has_succeeded = false;
        RCLCPP_ERROR(halPigpioNode->get_logger(),"Failed to configure GPIO %u as output!", request->gpioId);
    }
    return true;
}

void PigpioInit::setPullUp(hal_pigpio::hal_pigpioSetPullUp::Request request,
                           hal_pigpio::hal_pigpioSetPullUp::Response response)
{
    if (set_pull_up_down(pigpioHandle, request->gpioId, PI_PUD_UP) == 0)
    {
        response->has_succeeded = true;
        RCLCPP_INFO(halPigpioNode->get_logger(),"Sucessfully set pull-up resistor for GPIO %u.", request->gpioId);
    }
    else
    {
        response->has_succeeded = true;
        RCLCPP_INFO(halPigpioNode->get_logger(),"Failed to set pull-up resistor for GPIO %u!", request->gpioId);
    }

    return true;
}

void PigpioInit::setPullDown(hal_pigpio::hal_pigpioSetPullDown::Request request,
                             hal_pigpio::hal_pigpioSetPullDown::Response response)
{
    if (set_pull_up_down(pigpioHandle, request->gpioId, PI_PUD_DOWN) == 0)
    {
        response->has_succeeded = true;
        RCLCPP_INFO(halPigpioNode->get_logger(),"Sucessfully set pull-down resistor for GPIO %u.", request->gpioId);
    }
    else
    {
        response->has_succeeded = true;
        RCLCPP_INFO(halPigpioNode->get_logger(),"Failed to set pull-down resistor for GPIO %u!", request->gpioId);
    }

    return true;
}

void PigpioInit::clearResistor(hal_pigpio::hal_pigpioClearResistor::Request request,
                               hal_pigpio::hal_pigpioClearResistor::Response response)
{
    if (set_pull_up_down(pigpioHandle, request->gpioId, PI_PUD_OFF) == 0)
    {
        response->has_succeeded = true;
        RCLCPP_INFO(halPigpioNode->get_logger(),"Sucessfully clear resistor for GPIO %u.", request->gpioId);
    }
    else
    {
        response->has_succeeded = true;
        RCLCPP_INFO(halPigpioNode->get_logger(),"Failed to clear resistor for GPIO %u!", request->gpioId);
    }

    return true;
}

void PigpioInit::publishHeartbeat()
{
    hal_pigpio::hal_pigpioHeartbeatMsg heartbeat;
    heartbeat.isAlive = true;
    heartbeatPublisher.publish(heartbeat);
}