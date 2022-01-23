#include "ros/ros.h"

// Pigpio library
#include <pigpiod_if2.h>

// Services and messages headers (generated)
#include "hal_pigpio/hal_pigpioReadGpio.h"
#include "hal_pigpio/hal_pigpioSetCallback.h"
#include "hal_pigpio/hal_pigpioEdgeChangeMsg.h"

typedef std::function<void(int, unsigned, unsigned, uint32_t)> edgeCallback_t;

class PigpioInput
{
private:
    ros::Publisher gpioEdgeChangePub;
    ros::ServiceServer readGpioService;
    ros::ServiceServer setCallbackRisingEdgeService;
    int pigpio_handle;
    std::vector<uint> callbackList;
    void gpioEdgeChangeCallback(int handle, unsigned gpioId, unsigned edgeChangeType, uint32_t timeSinceBoot_us);

public:
    PigpioInput(ros::NodeHandle *node, int handle);
    ~PigpioInput();
    bool readGpio(hal_pigpio::hal_pigpioReadGpio::Request &req,
                  hal_pigpio::hal_pigpioReadGpio::Response &res);
    bool setCallback(hal_pigpio::hal_pigpioSetCallback::Request &req,
                     hal_pigpio::hal_pigpioSetCallback::Response &res);
};