#include "ros/ros.h"
#include "std_msgs/String.h"

// Pigpio library
#include <pigpiod_if2.h>

// Services headers (generated)
#include "hal_pigpio/hal_pigpioGetHandle.h"
#include "hal_pigpio/hal_pigpioI2cOpen.h"
#include "hal_pigpio/hal_pigpioI2cClose.h"

static int pigpio_handle;

bool i2cOpen(hal_pigpio::hal_pigpioI2cOpen::Request &req,
             hal_pigpio::hal_pigpioI2cOpen::Response &res)
{
    int i2cHandle = i2c_open(pigpio_handle, req.bus, req.address, 0);
    res.handle = i2cHandle;
    if (i2cHandle >= 0)
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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hal_pigpioI2c");
    ros::NodeHandle node;

    ros::ServiceClient getPigpioHandle = node.serviceClient<hal_pigpio::hal_pigpioGetHandle>("hal_pigpioGetHandle");
    hal_pigpio::hal_pigpioGetHandle pigpioHandleRequest;
    getPigpioHandle.call(pigpioHandleRequest);
    pigpio_handle = pigpioHandleRequest.response.handle;

    ros::ServiceServer i2cOpenService = node.advertiseService("hal_pigpioI2cOpen", i2cOpen);
    ros::ServiceServer i2cCloseService = node.advertiseService("hal_pigpioI2cClose", i2cClose);

    ros::spin();

    return 0;
}