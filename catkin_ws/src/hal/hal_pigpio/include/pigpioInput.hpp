#include "ros/ros.h"
#include "std_msgs/String.h"
#include <unordered_map>

// Pigpio library
#include <pigpiod_if2.h>

// Services headers (generated)
#include "hal_pigpio/hal_pigpioGetHandle.h"
#include "hal_pigpio/hal_pigpioReadGpio.h"
#include "hal_pigpio/hal_pigpioSetCallback.h"

struct gpioEdgesNumber
{
    uint32_t nbRisingEdges;
    uint32_t nbFallingEdges;
};

class PigpioInput
{
private:
    ros::Publisher gpioEdgeChangePub;
    ros::ServiceServer ReadGpioService;
    ros::ServiceServer SetCallbackRisingEdgeService;
    int pigpio_handle;
    static std::unordered_map<unsigned, gpioEdgesNumber> inputEdges;

public:
    PigpioInput(ros::NodeHandle *node, int handle)
    {
        pigpio_handle = handle;

        ros::Publisher gpioEdgeChangePub = node->advertise<std_msgs::String>("gpioEdgeChange", 1000);

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
        auto it = inputEdges.find(gpioId);
        gpioEdgesNumber nbInputEdges;

        if (it != inputEdges.end())
        {
            if (edgeChangeType == 0)
            {
                it->second.nbRisingEdges = it->second.nbFallingEdges++;
            }
            else
            {
                it->second.nbFallingEdges = it->second.nbRisingEdges++;
            }
        }
        else
        {
            if (edgeChangeType == 0)
            {
                nbInputEdges.nbRisingEdges = 0;
                nbInputEdges.nbFallingEdges = 1;
            }
            else
            {
                nbInputEdges.nbRisingEdges = 1;
                nbInputEdges.nbFallingEdges = 0;
            }
            inputEdges.insert({gpioId, nbInputEdges});
        }
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