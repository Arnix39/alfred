#include "ros/ros.h"
#include "std_msgs/String.h"

// Pigpio library
#include <pigpiod_if2.h>

// Services headers (generated)
#include "hal_pigpio/hal_pigpioGetHandle.h"
#include "hal_pigpio/hal_pigpioReadGpio.h"

static int pigpio_handle;

bool readGpio(hal_pigpio::hal_pigpioReadGpio::Request &req,
              hal_pigpio::hal_pigpioReadGpio::Response &res)
{
    res.level = gpio_read(pigpio_handle, req.gpioId);
    if (res.level != PI_BAD_GPIO)
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

    ros::ServiceServer ReadGpioService = node.advertiseService("hal_pigpioReadGpio", readGpio);

    ros::spin();

    return 0;
}