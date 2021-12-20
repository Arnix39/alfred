#include "ros/ros.h"
#include "std_msgs/String.h"
#include <pigpiod_if2.h>
#include "hal/hal_pigpioGetHandle.h"
#include "hal/hal_pigpioSetInputMode.h"
#include "hal/hal_pigpioSetOutputMode.h"
#include "hal/hal_pigpioSetPwmDutycycle.h"
#include "hal/hal_pigpioSetPwmFrequency.h"

static int pigpio_handle;

bool getHandle(hal::hal_pigpioGetHandle::Request &req,
               hal::hal_pigpioGetHandle::Response &res)
{
    res.handle = pigpio_handle;
    return true;
}

bool setInputMode(hal::hal_pigpioSetInputMode::Request &req,
                  hal::hal_pigpioSetInputMode::Response &res)
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

bool setOutputMode(hal::hal_pigpioSetOutputMode::Request &req,
                   hal::hal_pigpioSetOutputMode::Response &res)
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

bool setPwmDutycycle(hal::hal_pigpioSetPwmDutycycle::Request &req,
                     hal::hal_pigpioSetPwmDutycycle::Response &res)
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

bool setPwmFrequency(hal::hal_pigpioSetPwmFrequency::Request &req,
                     hal::hal_pigpioSetPwmFrequency::Response &res)
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

    ros::Rate loop_rate(10);
    pigpio_handle = pigpio_start(NULL, NULL);

    while (ros::ok())
    {
        ros::ServiceServer getHandleservice = node.advertiseService("hal_pigpioGetHandle", getHandle);
        ros::ServiceServer setInputModeservice = node.advertiseService("hal_pigpioSetInputMode", setInputMode);
        ros::ServiceServer setOutputModeservice = node.advertiseService("hal_pigpioSetOutputMode", setOutputMode);
        ros::ServiceServer setPwmDutycycleservice = node.advertiseService("hal_pigpioSetPwmDutycycle", setPwmDutycycle);
        ros::ServiceServer setPwmFrequencyservice = node.advertiseService("hal_pigpioSetPwmFrequency", setPwmFrequency);

        ros::spinOnce();
        loop_rate.sleep();
    }

    pigpio_stop(pigpio_handle);
    return 0;
}