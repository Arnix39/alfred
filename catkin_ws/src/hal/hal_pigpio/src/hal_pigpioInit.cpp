#include "hal_pigpioInit.hpp"

PigpioInit::PigpioInit(ros::NodeHandle *node)
{
    pigpio_handle = pigpio_start(NULL, NULL);

    if (pigpio_handle < 0)
    {
        ROS_ERROR("Failed to start pigpio daemon");
    }

    getHandleService = node->advertiseService("hal_pigpioGetHandle", &PigpioInit::getHandle, this);
    setInputModeService = node->advertiseService("hal_pigpioSetInputMode", &PigpioInit::setInputMode, this);
    setOutputModeService = node->advertiseService("hal_pigpioSetOutputMode", &PigpioInit::setOutputMode, this);
}

PigpioInit::~PigpioInit()
{
    pigpio_stop(pigpio_handle);
}

bool PigpioInit::getHandle(hal_pigpio::hal_pigpioGetHandle::Request &req,
                           hal_pigpio::hal_pigpioGetHandle::Response &res)
{
    res.handle = pigpio_handle;
    return true;
}

bool PigpioInit::setInputMode(hal_pigpio::hal_pigpioSetInputMode::Request &req,
                              hal_pigpio::hal_pigpioSetInputMode::Response &res)
{
    if (set_mode(pigpio_handle, req.gpioId, PI_INPUT) == 0)
    {
        ROS_INFO("GPIO %u configured as input.", req.gpioId);
    }
    else
    {
        ROS_ERROR("Failed to configure GPIO %u as input!", req.gpioId);
    }

    return true;
}

bool PigpioInit::setOutputMode(hal_pigpio::hal_pigpioSetOutputMode::Request &req,
                               hal_pigpio::hal_pigpioSetOutputMode::Response &res)
{
    if (set_mode(pigpio_handle, req.gpioId, PI_OUTPUT) == 0)
    {
        ROS_INFO("GPIO %u configured as output.", req.gpioId);
    }
    else
    {
        ROS_ERROR("Failed to configure GPIO %u as output!", req.gpioId);
    }
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hal_pigpioInit");
    ros::NodeHandle node;

    PigpioInit pigpioInit = PigpioInit(&node);

    ros::spin();

    return 0;
}