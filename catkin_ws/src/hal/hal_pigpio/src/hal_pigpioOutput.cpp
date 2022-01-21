#include "hal_pigpioOutput.hpp"

// Services headers (generated)
#include "hal_pigpio/hal_pigpioGetHandle.h"

PigpioOutput::PigpioOutput(ros::NodeHandle *node, int handle)
{
    pigpio_handle = handle;

    setPwmDutycycleService = node->advertiseService("hal_pigpioSetPwmDutycycle", &PigpioOutput::setPwmDutycycle, this);
    setPwmFrequencyService = node->advertiseService("hal_pigpioSetPwmFrequency", &PigpioOutput::setPwmFrequency, this);
    setGpioHighService = node->advertiseService("hal_pigpioSetGpioHigh", &PigpioOutput::setGpioHigh, this);
    setGpioLowService = node->advertiseService("hal_pigpioSetGpioLow", &PigpioOutput::setGpioLow, this);
}

bool PigpioOutput::setPwmDutycycle(hal_pigpio::hal_pigpioSetPwmDutycycle::Request &req,
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

bool PigpioOutput::setPwmFrequency(hal_pigpio::hal_pigpioSetPwmFrequency::Request &req,
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

bool PigpioOutput::setGpioHigh(hal_pigpio::hal_pigpioSetGpioHigh::Request &req,
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

bool PigpioOutput::setGpioLow(hal_pigpio::hal_pigpioSetGpioLow::Request &req,
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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hal_pigpioOutput");
    ros::NodeHandle node;
    int pigpio_handle;

    ros::ServiceClient getPigpioHandle = node.serviceClient<hal_pigpio::hal_pigpioGetHandle>("hal_pigpioGetHandle");
    hal_pigpio::hal_pigpioGetHandle pigpioHandleRequest;
    getPigpioHandle.call(pigpioHandleRequest);
    pigpio_handle = pigpioHandleRequest.response.handle;

    PigpioOutput pigpioOutput = PigpioOutput(&node, pigpio_handle);

    ros::spin();

    return 0;
}