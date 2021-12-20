#include "ros/ros.h"
#include "std_msgs/String.h"

// Pigpio library
#include <pigpiod_if2.h>

// Services headers (generated)
#include "hal_pigpio/hal_pigpioGetHandle.h"
#include "hal_pigpio/hal_pigpioSetInputMode.h"
#include "hal_pigpio/hal_pigpioSetOutputMode.h"

static int pigpio_handle;

bool getHandle(hal_pigpio::hal_pigpioGetHandle::Request &req,
               hal_pigpio::hal_pigpioGetHandle::Response &res)
{
    res.handle = pigpio_handle;
    return true;
}

bool setInputMode(hal_pigpio::hal_pigpioSetInputMode::Request &req,
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

bool setOutputMode(hal_pigpio::hal_pigpioSetOutputMode::Request &req,
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

    pigpio_handle = pigpio_start(NULL, NULL);

    if (pigpio_handle < 0)
    {
        ROS_ERROR("Failed to start PiGpio daemon");
        return 1;
    }

    ros::ServiceServer getHandleService = node.advertiseService("hal_pigpioGetHandle", getHandle);
    ros::ServiceServer setInputModeService = node.advertiseService("hal_pigpioSetInputMode", setInputMode);
    ros::ServiceServer setOutputModeService = node.advertiseService("hal_pigpioSetOutputMode", setOutputMode);

    ros::spin();

    pigpio_stop(pigpio_handle);
    return 0;
}