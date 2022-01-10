#include "pigpioI2c.hpp"

// Services headers (generated)
#include "hal_pigpio/hal_pigpioGetHandle.h"

PigpioI2c::PigpioI2c(ros::NodeHandle *node, int handle)
{
    pigpio_handle = handle;

    ros::ServiceServer i2cOpenService = node->advertiseService("hal_pigpioI2cOpen", &PigpioI2c::i2cOpen, this);
    ros::ServiceServer i2cCloseService = node->advertiseService("hal_pigpioI2cClose", &PigpioI2c::i2cClose, this);
}

bool PigpioI2c::i2cOpen(hal_pigpio::hal_pigpioI2cOpen::Request &req,
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

bool PigpioI2c::i2cClose(hal_pigpio::hal_pigpioI2cClose::Request &req,
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
    int pigpio_handle;

    ros::ServiceClient getPigpioHandle = node.serviceClient<hal_pigpio::hal_pigpioGetHandle>("hal_pigpioGetHandle");
    hal_pigpio::hal_pigpioGetHandle pigpioHandleRequest;
    getPigpioHandle.call(pigpioHandleRequest);
    pigpio_handle = pigpioHandleRequest.response.handle;

    PigpioI2c pigpioI2c = PigpioI2c(&node, pigpio_handle);

    ros::spin();

    return 0;
}