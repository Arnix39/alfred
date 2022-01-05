#include "ros/ros.h"
#include "std_msgs/String.h"

// Pigpio library
#include <pigpiod_if2.h>

// Services headers (generated)
#include "hal_pigpio/hal_pigpioGetHandle.h"
#include "hal_pigpio/hal_pigpioReadGpio.h"
#include "hal_pigpio/hal_pigpioSetCallback.h"

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

void gpioEdgeChangeCallback(int handle, unsigned gpioId, unsigned edgeChangeType, uint32_t timeSinceBoot_us)
{
}

bool setCallback(hal_pigpio::hal_pigpioSetCallback::Request &req,
                 hal_pigpio::hal_pigpioSetCallback::Response &res)
{
    res.callbackId = callback(pigpio_handle, req.gpioId, req.edgeChangeType, gpioEdgeChangeCallback);
    if (res.callbackId >= 0)
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
    ros::init(argc, argv, "hal_pigpioInput");
    ros::NodeHandle node;

    ros::ServiceClient getPigpioHandle = node.serviceClient<hal_pigpio::hal_pigpioGetHandle>("hal_pigpioGetHandle");
    hal_pigpio::hal_pigpioGetHandle pigpioHandleRequest;
    getPigpioHandle.call(pigpioHandleRequest);
    pigpio_handle = pigpioHandleRequest.response.handle;

    ros::ServiceServer ReadGpioService = node.advertiseService("hal_pigpioReadGpio", readGpio);
    ros::ServiceServer SetCallbackRisingEdgeService = node.advertiseService("hal_pigpioSetCallback", setCallback);

    ros::spin();

    return 0;
}