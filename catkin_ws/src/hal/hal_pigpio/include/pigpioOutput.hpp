#include "ros/ros.h"

// Pigpio library
#include <pigpiod_if2.h>

// Services headers (generated)
#include "hal_pigpio/hal_pigpioGetHandle.h"
#include "hal_pigpio/hal_pigpioSetPwmDutycycle.h"
#include "hal_pigpio/hal_pigpioSetPwmFrequency.h"
#include "hal_pigpio/hal_pigpioSetGpioHigh.h"
#include "hal_pigpio/hal_pigpioSetGpioLow.h"

class PigpioOutput
{
private:
    ros::ServiceServer setPwmDutycycleService;
    ros::ServiceServer setPwmFrequencyService;
    ros::ServiceServer setGpioHighService;
    ros::ServiceServer setGpioLowService;
    int pigpio_handle;

public:
    PigpioOutput(ros::NodeHandle *node, int handle)
    {
        pigpio_handle = handle;

        ros::ServiceServer setPwmDutycycleService = node->advertiseService("hal_pigpioSetPwmDutycycle", &PigpioOutput::setPwmDutycycle, this);
        ros::ServiceServer setPwmFrequencyService = node->advertiseService("hal_pigpioSetPwmFrequency", &PigpioOutput::setPwmFrequency, this);
        ros::ServiceServer setGpioHighService = node->advertiseService("hal_pigpioSetGpioHigh", &PigpioOutput::setGpioHigh, this);
        ros::ServiceServer setGpioLowService = node->advertiseService("hal_pigpioSetGpioLow", &PigpioOutput::setGpioLow, this);
    }

    bool setPwmDutycycle(hal_pigpio::hal_pigpioSetPwmDutycycle::Request &req,
                         hal_pigpio::hal_pigpioSetPwmDutycycle::Response &res)
    {
        if ((req.dutycycle != 0) && (set_PWM_dutycycle(pigpio_handle, req.gpioId, req.dutycycle) == 0))
        {
            res.result = true;
        }
        else
        {
            res.result = false;
        }
        return true;
    }

    bool setPwmFrequency(hal_pigpio::hal_pigpioSetPwmFrequency::Request &req,
                         hal_pigpio::hal_pigpioSetPwmFrequency::Response &res)
    {
        int pwmSettingResult = set_PWM_dutycycle(pigpio_handle, req.gpioId, req.frequency);

        if ((pwmSettingResult != PI_NOT_PERMITTED) && (pwmSettingResult != PI_BAD_USER_GPIO))
        {
            res.result = true;
        }
        else
        {
            res.result = false;
        }
        return true;
    }

    bool setGpioHigh(hal_pigpio::hal_pigpioSetGpioHigh::Request &req,
                     hal_pigpio::hal_pigpioSetGpioHigh::Response &res)
    {
        if ((gpio_write(pigpio_handle, req.gpioId, PI_HIGH) != 0))
        {
            res.result = true;
        }
        else
        {
            res.result = false;
        }
        return true;
    }

    bool setGpioLow(hal_pigpio::hal_pigpioSetGpioLow::Request &req,
                    hal_pigpio::hal_pigpioSetGpioLow::Response &res)
    {
        if ((gpio_write(pigpio_handle, req.gpioId, PI_LOW) != 0))
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