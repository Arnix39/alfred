#include "ros/ros.h"

// Pigpio library
#include <pigpiod_if2.h>

// Services and messages headers (generated)
#include "hal_pigpio/hal_pigpioGetHandle.h"
#include "hal_pigpio/hal_pigpioSetInputMode.h"
#include "hal_pigpio/hal_pigpioSetOutputMode.h"

class PigpioInit
{
private:
    ros::ServiceServer i2cOpenService;
    ros::ServiceServer i2cCloseService;
    int pigpio_handle;

public:
    PigpioInit(ros::NodeHandle *node);
    ~PigpioInit();
    bool getHandle(hal_pigpio::hal_pigpioGetHandle::Request &req,
                   hal_pigpio::hal_pigpioGetHandle::Response &res);
    bool setInputMode(hal_pigpio::hal_pigpioSetInputMode::Request &req,
                      hal_pigpio::hal_pigpioSetInputMode::Response &res);
    bool setOutputMode(hal_pigpio::hal_pigpioSetOutputMode::Request &req,
                       hal_pigpio::hal_pigpioSetOutputMode::Response &res);
};