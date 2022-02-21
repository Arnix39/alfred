#ifndef HAL_PIGPIOINIT
#define HAL_PIGPIOINIT

#include "ros/ros.h"

// Pigpio library
#include <pigpiod_if2.h>

// Services and messages headers (generated)
#include "hal_pigpio/hal_pigpioGetHandle.h"
#include "hal_pigpio/hal_pigpioGetMode.h"
#include "hal_pigpio/hal_pigpioSetInputMode.h"
#include "hal_pigpio/hal_pigpioSetOutputMode.h"

class PigpioInit
{
private:
    ros::ServiceServer getHandleService;
    ros::ServiceServer getModeService;
    ros::ServiceServer setInputModeService;
    ros::ServiceServer setOutputModeService;
    int pigpioHandle;

public:
    PigpioInit(ros::NodeHandle *node, int pigpioHandle);
    ~PigpioInit();
    bool getHandle(hal_pigpio::hal_pigpioGetHandle::Request &req,
                   hal_pigpio::hal_pigpioGetHandle::Response &res);
    bool getMode(hal_pigpio::hal_pigpioGetMode::Request &req,
                 hal_pigpio::hal_pigpioGetMode::Response &res);
    bool setInputMode(hal_pigpio::hal_pigpioSetInputMode::Request &req,
                      hal_pigpio::hal_pigpioSetInputMode::Response &res);
    bool setOutputMode(hal_pigpio::hal_pigpioSetOutputMode::Request &req,
                       hal_pigpio::hal_pigpioSetOutputMode::Response &res);
};

#endif