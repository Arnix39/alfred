#ifndef HAL_PIGPIOOUTPUT
#define HAL_PIGPIOOUTPUT

#include "ros/ros.h"

// Pigpio library
#include <pigpiod_if2.h>

// Services and messages headers (generated)
#include "hal_pigpio/hal_pigpioSetPwmDutycycle.h"
#include "hal_pigpio/hal_pigpioSetPwmFrequency.h"
#include "hal_pigpio/hal_pigpioSetGpioHigh.h"
#include "hal_pigpio/hal_pigpioSetGpioLow.h"
#include "hal_pigpio/hal_pigpioSendTriggerPulse.h"
#include "hal_pigpio/hal_pigpioGetMode.h"

class PigpioOutput
{
private:
    ros::ServiceServer setPwmDutycycleService;
    ros::ServiceServer setPwmFrequencyService;
    ros::ServiceServer setGpioHighService;
    ros::ServiceServer setGpioLowService;
    ros::ServiceServer sendTriggerPulseService;
    ros::ServiceClient getModeClient;
    int pigpioHandle;

public:
    PigpioOutput(ros::NodeHandle *node, int handle);
    ~PigpioOutput() = default;
    bool setPwmDutycycle(hal_pigpio::hal_pigpioSetPwmDutycycle::Request &req,
                         hal_pigpio::hal_pigpioSetPwmDutycycle::Response &res);
    bool setPwmFrequency(hal_pigpio::hal_pigpioSetPwmFrequency::Request &req,
                         hal_pigpio::hal_pigpioSetPwmFrequency::Response &res);
    bool setGpioHigh(hal_pigpio::hal_pigpioSetGpioHigh::Request &req,
                     hal_pigpio::hal_pigpioSetGpioHigh::Response &res);
    bool setGpioLow(hal_pigpio::hal_pigpioSetGpioLow::Request &req,
                    hal_pigpio::hal_pigpioSetGpioLow::Response &res);
    bool sendTriggerPulse(hal_pigpio::hal_pigpioSendTriggerPulse::Request &req,
                          hal_pigpio::hal_pigpioSendTriggerPulse::Response &res);
};

#endif