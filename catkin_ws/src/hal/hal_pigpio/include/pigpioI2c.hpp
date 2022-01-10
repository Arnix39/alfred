#include "ros/ros.h"

// Pigpio library
#include <pigpiod_if2.h>

// Services headers (generated)
#include "hal_pigpio/hal_pigpioI2cOpen.h"
#include "hal_pigpio/hal_pigpioI2cClose.h"

class PigpioI2c
{
private:
    ros::ServiceServer i2cOpenService;
    ros::ServiceServer i2cCloseService;
    int pigpio_handle;

public:
    PigpioI2c(ros::NodeHandle *node, int handle)
    {
        pigpio_handle = handle;

        ros::ServiceServer i2cOpenService = node->advertiseService("hal_pigpioI2cOpen", &PigpioI2c::i2cOpen, this);
        ros::ServiceServer i2cCloseService = node->advertiseService("hal_pigpioI2cClose", &PigpioI2c::i2cClose, this);
    }

    bool i2cOpen(hal_pigpio::hal_pigpioI2cOpen::Request &req,
                 hal_pigpio::hal_pigpioI2cOpen::Response &res)
    {
        res.handle = i2c_open(pigpio_handle, req.bus, req.address, 0);
        if (res.handle >= 0)
        {
            res.result = true;
        }
        else
        {
            res.result = false;
        }
        return true;
    }

    bool i2cClose(hal_pigpio::hal_pigpioI2cClose::Request &req,
                  hal_pigpio::hal_pigpioI2cClose::Response &res)
    {
        if (i2c_close(pigpio_handle, req.handle) == 0)
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