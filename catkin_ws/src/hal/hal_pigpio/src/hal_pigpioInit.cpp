#include "pigpioInit.hpp"

PigpioInit::PigpioInit(ros::NodeHandle *node)
{
    pigpio_handle = pigpio_start(NULL, NULL);

    if (pigpio_handle < 0)
    {
        ROS_ERROR("Failed to start pigpio daemon");
    }

    ros::ServiceServer getHandleService = node->advertiseService("hal_pigpioGetHandle", &PigpioInit::getHandle, this);
    ros::ServiceServer setInputModeService = node->advertiseService("hal_pigpioSetInputMode", &PigpioInit::setInputMode, this);
    ros::ServiceServer setOutputModeService = node->advertiseService("hal_pigpioSetOutputMode", &PigpioInit::setOutputMode, this);
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
        res.result = true;
    }
    else
    {
        res.result = false;
    }

    return true;
}

bool PigpioInit::setOutputMode(hal_pigpio::hal_pigpioSetOutputMode::Request &req,
                               hal_pigpio::hal_pigpioSetOutputMode::Response &res)
{
    if (set_mode(pigpio_handle, req.gpioId, PI_OUTPUT) == 0)
    {
        res.result = true;
    }
    else
    {
        res.result = false;
    }
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hal_pigpioInit");
    ros::NodeHandle node;

    ros::spin();

    return 0;
}