#include "ros/ros.h"
#include "std_msgs/String.h"

// Pigpio library
#include <pigpiod_if2.h>

// Services headers (generated)
#include "hal_pigpio/hal_pigpioGetHandle.h"
#include "hal_pigpio/hal_pigpioSetInputMode.h"
#include "hal_pigpio/hal_pigpioSetOutputMode.h"
#include "hal_pigpio/hal_pigpioSetPwmDutycycle.h"
#include "hal_pigpio/hal_pigpioSetPwmFrequency.h"

static int pigpio_handle;

bool getHandle(hal_pigpio::hal_pigpioGetHandle::Request &req,
               hal_pigpio::hal_pigpioGetHandle::Response &res)
{
    res.handle = pigpio_handle;
    return true;
}

bool setInputMode(hal_pigpio::hal_pigpioSetInputMode::Request &req,
                  hal_pigpio::hal_pigpioSetInputMode::Response &res)
{
    if (set_mode(pigpio_handle, req.gpioId, PI_INPUT) == 0)
    {
        res.result = true;
    }
    else
    {
        res.result = false;
    }

    return true;
}

bool setOutputMode(hal_pigpio::hal_pigpioSetOutputMode::Request &req,
                   hal_pigpio::hal_pigpioSetOutputMode::Response &res)
{
    if (set_mode(pigpio_handle, req.gpioId, PI_OUTPUT) == 0)
    {
        res.result = true;
    }
    else
    {
        res.result = false;
    }
    return true;
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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hal_pigpioInit");
    ros::NodeHandle node;

    ros::Rate loop_rate(1000);
    pigpio_handle = pigpio_start(NULL, NULL);

    ros::ServiceServer getHandleService = node.advertiseService("hal_pigpioGetHandle", getHandle);
    ros::ServiceServer setInputModeService = node.advertiseService("hal_pigpioSetInputMode", setInputMode);
    ros::ServiceServer setOutputModeService = node.advertiseService("hal_pigpioSetOutputMode", setOutputMode);
    ros::ServiceServer setPwmDutycycleService = node.advertiseService("hal_pigpioSetPwmDutycycle", setPwmDutycycle);
    ros::ServiceServer setPwmFrequencyService = node.advertiseService("hal_pigpioSetPwmFrequency", setPwmFrequency);

    ros::spin();

    pigpio_stop(pigpio_handle);
    return 0;
}