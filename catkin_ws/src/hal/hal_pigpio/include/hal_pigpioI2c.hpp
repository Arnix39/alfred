#ifndef HAL_PIGPIOI2C
#define HAL_PIGPIOI2C

#include "ros/ros.h"

// Pigpio library
#include <pigpiod_if2.h>

// Services and messages headers (generated)
#include "hal_pigpio/hal_pigpioI2cOpen.h"
#include "hal_pigpio/hal_pigpioI2cClose.h"

class PigpioI2c
{
private:
    ros::ServiceServer i2cOpenService;
    ros::ServiceServer i2cCloseService;
    int pigpio_handle;

public:
    PigpioI2c(ros::NodeHandle *node, int handle);
    ~PigpioI2c() = default;
    bool i2cOpen(hal_pigpio::hal_pigpioI2cOpen::Request &req,
                 hal_pigpio::hal_pigpioI2cOpen::Response &res);
    bool i2cClose(hal_pigpio::hal_pigpioI2cClose::Request &req,
                  hal_pigpio::hal_pigpioI2cClose::Response &res);
};

#endif