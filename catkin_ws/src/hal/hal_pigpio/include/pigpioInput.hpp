#include "ros/ros.h"

// Pigpio library
#include <pigpiod_if2.h>

// Services headers (generated)
#include "hal_pigpio/hal_pigpioGetHandle.h"
#include "hal_pigpio/hal_pigpioReadGpio.h"
#include "hal_pigpio/hal_pigpioSetCallback.h"
#include "hal_pigpio/hal_pigpioGetEdgesNumber.h"
#include "hal_pigpio/hal_pigpioResetEdgesNumber.h"
#include "hal_pigpio/hal_pgpioEdgeChangeMsg.h"

class PigpioInput
{
private:
    inline static ros::Publisher gpioEdgeChangePub;
    ros::ServiceServer ReadGpioService;
    ros::ServiceServer SetCallbackRisingEdgeService;
    int pigpio_handle;

public:
    PigpioInput(ros::NodeHandle *node, int handle)
    {
        pigpio_handle = handle;

        ros::Publisher gpioEdgeChangePub = node->advertise<hal_pigpio::hal_pgpioEdgeChangeMsg>("gpioEdgeChange", 1000);

        ros::ServiceServer ReadGpioService = node->advertiseService("hal_pigpioReadGpio", &PigpioInput::readGpio, this);
        ros::ServiceServer SetCallbackRisingEdgeService = node->advertiseService("hal_pigpioSetCallback", &PigpioInput::setCallback, this);
    }

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

    static void gpioEdgeChangeCallback(int handle, unsigned gpioId, unsigned edgeChangeType, uint32_t timeSinceBoot_us)
    {
        hal_pigpio::hal_pgpioEdgeChangeMsg edgeChangeMsg;

        edgeChangeMsg.gpioId = gpioId;
        edgeChangeMsg.edgeChangeType = edgeChangeType;
        edgeChangeMsg.timeSinceBoot_us = timeSinceBoot_us;

        gpioEdgeChangePub.publish(edgeChangeMsg);
    }

    bool setCallback(hal_pigpio::hal_pigpioSetCallback::Request &req,
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
};