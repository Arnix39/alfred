#include "hal_pigpioInput.hpp"

// Services headers (generated)
#include "hal_pigpio/hal_pigpioGetHandle.h"

PigpioInput::PigpioInput(ros::NodeHandle *node, int pigpioHandle) : pigpioHandle(pigpioHandle),
                                                                    getPigpioHandleClient(node->serviceClient<hal_pigpio::hal_pigpioGetHandle>("hal_pigpioGetHandle")),
                                                                    gpioEdgeChangePub(node->advertise<hal_pigpio::hal_pigpioEdgeChangeMsg>("gpioEdgeChange", 1000))
{
    ROS_INFO("Pigpio Input object created.");

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
        ROS_INFO("Read GPIO %u.", req.gpioId);
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
void c_callback_wrapper(int handle, unsigned gpioId, unsigned edgeChangeType, uint32_t timeSinceBoot_us, void *userdata)
{
    auto &c_edgeCallback = *reinterpret_cast<edgeCallback_t *>(handle, gpioId, edgeChangeType, timeSinceBoot_us, userdata);
    c_edgeCallback(handle, gpioId, edgeChangeType, timeSinceBoot_us);
}

bool PigpioInput::setCallback(hal_pigpio::hal_pigpioSetCallback::Request &req,
                              hal_pigpio::hal_pigpioSetCallback::Response &res)
{
    edgeCallback_t edgeCallback = std::bind(&PigpioInput::gpioEdgeChangeCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4);
    res.callbackId = callback_ex(pigpioHandle, req.gpioId, req.edgeChangeType, c_callback_wrapper, &edgeCallback);
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