#include "hal_pigpioInput.hpp"

// Services headers (generated)
#include "hal_pigpio/hal_pigpioGetHandle.h"

PigpioInput::PigpioInput(ros::NodeHandle *node, int handle)
{
    pigpio_handle = handle;

    ros::Publisher gpioEdgeChangePub = node->advertise<hal_pigpio::hal_pigpioEdgeChangeMsg>("gpioEdgeChange", 1000);

    ros::ServiceServer readGpioService = node->advertiseService("hal_pigpioReadGpio", &PigpioInput::readGpio, this);
    ros::ServiceServer setCallbackRisingEdgeService = node->advertiseService("hal_pigpioSetCallback", &PigpioInput::setCallback, this);
}

bool PigpioInput::readGpio(hal_pigpio::hal_pigpioReadGpio::Request &req,
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

void PigpioInput::gpioEdgeChangeCallback(int handle, unsigned gpioId, unsigned edgeChangeType, uint32_t timeSinceBoot_us)
{
    hal_pigpio::hal_pigpioEdgeChangeMsg edgeChangeMsg;

    edgeChangeMsg.gpioId = gpioId;
    edgeChangeMsg.edgeChangeType = edgeChangeType;
    edgeChangeMsg.timeSinceBoot_us = timeSinceBoot_us;

    gpioEdgeChangePub.publish(edgeChangeMsg);
}

bool PigpioInput::setCallback(hal_pigpio::hal_pigpioSetCallback::Request &req,
                              hal_pigpio::hal_pigpioSetCallback::Response &res)
{
    res.callbackId = callback(pigpio_handle, req.gpioId, req.edgeChangeType, PigpioInput::gpioEdgeChangeCallback);
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
    int pigpio_handle;

    ros::ServiceClient getPigpioHandle = node.serviceClient<hal_pigpio::hal_pigpioGetHandle>("hal_pigpioGetHandle");
    hal_pigpio::hal_pigpioGetHandle pigpioHandleRequest;
    getPigpioHandle.call(pigpioHandleRequest);
    pigpio_handle = pigpioHandleRequest.response.handle;

    PigpioInput pigpioInput = PigpioInput(&node, pigpio_handle);

    ros::spin();

    return 0;
}