#include "hal_pigpioInput.hpp"

// Services headers (generated)
#include "hal_pigpio/hal_pigpioGetHandle.h"

PigpioInput::PigpioInput(ros::NodeHandle *node, int pigpioHandle) : pigpioHandle(pigpioHandle),
                                                                    getPigpioHandleClient(node->serviceClient<hal_pigpio::hal_pigpioGetHandle>("hal_pigpioGetHandle")),
                                                                    gpioEdgeChangePub(node->advertise<hal_pigpio::hal_pigpioEdgeChangeMsg>("gpioEdgeChange", 1000))
{
    readGpioService = node->advertiseService("hal_pigpioReadGpio", &PigpioInput::readGpio, this);
    setCallbackRisingEdgeService = node->advertiseService("hal_pigpioSetCallback", &PigpioInput::setCallback, this);
}

PigpioInput::~PigpioInput()
{
    for (uint callbackId : callbackList)
    {
        callback_cancel(callbackId);
    }
}

bool PigpioInput::readGpio(hal_pigpio::hal_pigpioReadGpio::Request &req,
                           hal_pigpio::hal_pigpioReadGpio::Response &res)
{
    res.level = gpio_read(pigpioHandle, req.gpioId);
    if (res.level != PI_BAD_GPIO)
    {
        res.hasSucceeded = true;
    }
    else
    {
        res.hasSucceeded = false;
        ROS_INFO("Failed to read GPIO %u!", req.gpioId);
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

// This is to pass the pointer of the callback function gpioEdgeChangeCallback to the pigpio library API
void PigpioInput::c_gpioEdgeChangeCallback(int handle, unsigned gpioId, unsigned edgeChangeType, uint32_t timeSinceBoot_us, void *userData)
{
    PigpioInput *object = reinterpret_cast<PigpioInput *>(userData);
    object->gpioEdgeChangeCallback(handle, gpioId, edgeChangeType, timeSinceBoot_us);
}

bool PigpioInput::setCallback(hal_pigpio::hal_pigpioSetCallback::Request &req,
                              hal_pigpio::hal_pigpioSetCallback::Response &res)
{
    res.callbackId = callback_ex(pigpioHandle, req.gpioId, req.edgeChangeType, c_gpioEdgeChangeCallback, reinterpret_cast<void *>(this));
    if (res.callbackId >= 0)
    {
        res.hasSucceeded = true;
        callbackList.push_back((uint)res.callbackId);
        ROS_INFO("Callback for GPIO %u configured.", req.gpioId);
    }
    else
    {
        res.hasSucceeded = false;
        ROS_ERROR("Failed to configure callback for GPIO %u!", req.gpioId);
    }
    return true;
}