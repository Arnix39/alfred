#include "ros/ros.h"

// Pigpio library
#include <pigpiod_if2.h>

// Services headers (generated)
#include "hal_pigpio/hal_pigpioReadGpio.h"
#include "hal_pigpio/hal_pigpioSetCallback.h"
#include "hal_pigpio/hal_pigpioGetEdgesNumber.h"
#include "hal_pigpio/hal_pigpioResetEdgesNumber.h"
#include "hal_pigpio/hal_pgpioEdgeChangeMsg.h"

class PigpioInput
{
private:
    inline static ros::Publisher gpioEdgeChangePub;
    ros::ServiceServer readGpioService;
    ros::ServiceServer setCallbackRisingEdgeService;
    int pigpio_handle;

public:
    PigpioInput(ros::NodeHandle *node, int handle);
    bool readGpio(hal_pigpio::hal_pigpioReadGpio::Request &req,
                  hal_pigpio::hal_pigpioReadGpio::Response &res);
    static void gpioEdgeChangeCallback(int handle, unsigned gpioId, unsigned edgeChangeType, uint32_t timeSinceBoot_us);
    bool setCallback(hal_pigpio::hal_pigpioSetCallback::Request &req,
                     hal_pigpio::hal_pigpioSetCallback::Response &res);
};