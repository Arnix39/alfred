#include "hal_pigpioInit.hpp"

PigpioInit::PigpioInit(ros::NodeHandle *node)
{
    pigpioHandle = pigpio_start(NULL, NULL);

    if (pigpioHandle < 0)
    {
        ROS_ERROR("Failed to start pigpio daemon!");
    }

    ROS_INFO("Pigpio handle: %d.", pigpioHandle);

    getHandleService = node->advertiseService("hal_pigpioGetHandle", &PigpioInit::getHandle, this);
    setInputModeService = node->advertiseService("hal_pigpioSetInputMode", &PigpioInit::setInputMode, this);
    setOutputModeService = node->advertiseService("hal_pigpioSetOutputMode", &PigpioInit::setOutputMode, this);
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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hal_pigpioInit");
    ros::NodeHandle node;

    PigpioInit pigpioInit(&node);

    ros::spin();

    return 0;
}