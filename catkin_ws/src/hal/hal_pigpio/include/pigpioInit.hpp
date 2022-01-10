#include "ros/ros.h"

// Pigpio library
#include <pigpiod_if2.h>

// Services headers (generated)
#include "hal_pigpio/hal_pigpioGetHandle.h"
#include "hal_pigpio/hal_pigpioSetInputMode.h"
#include "hal_pigpio/hal_pigpioSetOutputMode.h"

class PigpioInit
{
private:
    ros::ServiceServer i2cOpenService;
    ros::ServiceServer i2cCloseService;
    int pigpio_handle;

public:
    PigpioInit(ros::NodeHandle *node)
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

    ~PigpioInit()
    {
        pigpio_stop(pigpio_handle);
    }

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
};