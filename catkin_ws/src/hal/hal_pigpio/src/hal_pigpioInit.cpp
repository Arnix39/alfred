#include "hal_pigpioInit.hpp"

PigpioInit::PigpioInit(ros::NodeHandle *node, int pigpioHandle) : pigpioHandle(pigpioHandle),
                                                                  getHandleService(node->advertiseService("hal_pigpioGetHandle", &PigpioInit::getHandle, this)),
                                                                  getModeService(node->advertiseService("hal_pigpioGetMode", &PigpioInit::getMode, this)),
                                                                  setInputModeService(node->advertiseService("hal_pigpioSetInputMode", &PigpioInit::setInputMode, this)),
                                                                  setOutputModeService(node->advertiseService("hal_pigpioSetOutputMode", &PigpioInit::setOutputMode, this)),
                                                                  setPullUpService(node->advertiseService("hal_pigpioSetPullUp", &PigpioInit::setPullUp, this)),
                                                                  setPullDownService(node->advertiseService("hal_pigpioSetPullDown", &PigpioInit::setPullDown, this)),
                                                                  clearResistorService(node->advertiseService("hal_pigpioClearResistor", &PigpioInit::clearResistor, this))
{
}

PigpioInit::~PigpioInit()
{
    ROS_INFO("Stopping pigpio daemon.");
    pigpio_stop(pigpioHandle);
}

bool PigpioInit::getHandle(hal_pigpio::hal_pigpioGetHandle::Request &req,
                           hal_pigpio::hal_pigpioGetHandle::Response &res)
{
    res.handle = pigpioHandle;
    return true;
}

bool PigpioInit::getMode(hal_pigpio::hal_pigpioGetMode::Request &req,
                         hal_pigpio::hal_pigpioGetMode::Response &res)
{
    res.mode = get_mode(pigpioHandle, req.gpioId);
    if (res.mode >= 0)
    {
        res.hasSucceeded = true;
    }
    else
    {
        res.hasSucceeded = false;
        ROS_ERROR("Failed to retrieve mode for GPIO %u!", req.gpioId);
    }
    return true;
}

bool PigpioInit::setInputMode(hal_pigpio::hal_pigpioSetInputMode::Request &req,
                              hal_pigpio::hal_pigpioSetInputMode::Response &res)
{
    if (set_mode(pigpioHandle, req.gpioId, PI_INPUT) == 0)
    {
        res.hasSucceeded = true;
        ROS_INFO("GPIO %u configured as input.", req.gpioId);
    }
    else
    {
        res.hasSucceeded = false;
        ROS_ERROR("Failed to configure GPIO %u as input!", req.gpioId);
    }

    return true;
}

bool PigpioInit::setOutputMode(hal_pigpio::hal_pigpioSetOutputMode::Request &req,
                               hal_pigpio::hal_pigpioSetOutputMode::Response &res)
{
    if (set_mode(pigpioHandle, req.gpioId, PI_OUTPUT) == 0)
    {
        res.hasSucceeded = true;
        ROS_INFO("GPIO %u configured as output.", req.gpioId);
    }
    else
    {
        res.hasSucceeded = false;
        ROS_ERROR("Failed to configure GPIO %u as output!", req.gpioId);
    }
    return true;
}

bool PigpioInit::setPullUp(hal_pigpio::hal_pigpioSetPullUp::Request &req,
                           hal_pigpio::hal_pigpioSetPullUp::Response &res)
{
    if (set_pull_up_down(pigpioHandle, req.gpioId, PI_PUD_UP) == 0)
    {
        res.hasSucceeded = true;
        ROS_INFO("Sucessfully set pull-up resistor for GPIO %u.", req.gpioId);
    }
    else
    {
        res.hasSucceeded = true;
        ROS_INFO("Failed to set pull-up resistor for GPIO %u!", req.gpioId);
    }

    return true;
}

bool PigpioInit::setPullDown(hal_pigpio::hal_pigpioSetPullDown::Request &req,
                             hal_pigpio::hal_pigpioSetPullDown::Response &res)
{
    if (set_pull_up_down(pigpioHandle, req.gpioId, PI_PUD_DOWN) == 0)
    {
        res.hasSucceeded = true;
        ROS_INFO("Sucessfully set pull-down resistor for GPIO %u.", req.gpioId);
    }
    else
    {
        res.hasSucceeded = true;
        ROS_INFO("Failed to set pull-down resistor for GPIO %u!", req.gpioId);
    }

    return true;
}

bool PigpioInit::clearResistor(hal_pigpio::hal_pigpioClearResistor::Request &req,
                               hal_pigpio::hal_pigpioClearResistor::Response &res)
{
    if (set_pull_up_down(pigpioHandle, req.gpioId, PI_PUD_OFF) == 0)
    {
        res.hasSucceeded = true;
        ROS_INFO("Sucessfully clear resistor for GPIO %u.", req.gpioId);
    }
    else
    {
        res.hasSucceeded = true;
        ROS_INFO("Failed to clear resistor for GPIO %u!", req.gpioId);
    }

    return true;
}