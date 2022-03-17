#ifndef HAL_PIGPIO_IMU
#define HAL_PIGPIO_IMU

#include "ros/ros.h"

// Pigpio library
#include <pigpiod_if2.h>

class PigpioImu
{
private:
    int pigpioHandle;

public:
    PigpioImu(ros::NodeHandle *node, int pigpioHandle);
    ~PigpioImu() = default;
};

#endif