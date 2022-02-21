#include "hal_pigpioOutput.hpp"

// Services headers (generated)
#include "hal_pigpio/hal_pigpioGetHandle.h"

PigpioOutput::PigpioOutput(ros::NodeHandle *node, int handle) : pigpioHandle(handle),
                                                                getModeClient(node->serviceClient<hal_pigpio::hal_pigpioGetMode>("hal_pigpioGetMode"))
{
    setPwmDutycycleService = node->advertiseService("hal_pigpioSetPwmDutycycle", &PigpioOutput::setPwmDutycycle, this);
    setPwmFrequencyService = node->advertiseService("hal_pigpioSetPwmFrequency", &PigpioOutput::setPwmFrequency, this);
    setGpioHighService = node->advertiseService("hal_pigpioSetGpioHigh", &PigpioOutput::setGpioHigh, this);
    setGpioLowService = node->advertiseService("hal_pigpioSetGpioLow", &PigpioOutput::setGpioLow, this);
    sendTriggerPulseService = node->advertiseService("hal_pigpioSendTriggerPulse", &PigpioOutput::sendTriggerPulse, this);
}

bool PigpioOutput::setPwmDutycycle(hal_pigpio::hal_pigpioSetPwmDutycycle::Request &req,
                                   hal_pigpio::hal_pigpioSetPwmDutycycle::Response &res)
{
    if ((req.dutycycle != 0) && (set_PWM_dutycycle(pigpioHandle, req.gpioId, req.dutycycle) == 0))
    {
        res.hasSucceeded = true;
        ROS_INFO("Set PWM duty cycle of %u for GPIO %u.", req.dutycycle, req.gpioId);
    }
    else
    {
        res.hasSucceeded = false;
        ROS_ERROR("Failed to set PWM duty cycle for GPIO %u!", req.gpioId);
    }
    return true;
}

bool PigpioOutput::setPwmFrequency(hal_pigpio::hal_pigpioSetPwmFrequency::Request &req,
                                   hal_pigpio::hal_pigpioSetPwmFrequency::Response &res)
{
    int pwmSettingResult = set_PWM_dutycycle(pigpioHandle, req.gpioId, req.frequency);

    if ((pwmSettingResult != PI_NOT_PERMITTED) && (pwmSettingResult != PI_BAD_USER_GPIO))
    {
        res.hasSucceeded = true;
        ROS_INFO("Set PWM frequency of %u for GPIO %u.", req.frequency, req.gpioId);
    }
    else
    {
        res.hasSucceeded = false;
        ROS_ERROR("Failed to set PWM frequency for GPIO %u!", req.gpioId);
    }
    return true;
}

bool PigpioOutput::setGpioHigh(hal_pigpio::hal_pigpioSetGpioHigh::Request &req,
                               hal_pigpio::hal_pigpioSetGpioHigh::Response &res)
{
    if (gpio_write(pigpioHandle, req.gpioId, PI_HIGH) == 0)
    {
        res.hasSucceeded = true;
        ROS_INFO("Set GPIO %u to high level.", req.gpioId);
    }
    else
    {
        res.hasSucceeded = false;
        ROS_ERROR("Failed to set GPIO %u to high level!", req.gpioId);
    }
    return true;
}

bool PigpioOutput::setGpioLow(hal_pigpio::hal_pigpioSetGpioLow::Request &req,
                              hal_pigpio::hal_pigpioSetGpioLow::Response &res)
{
    if (gpio_write(pigpioHandle, req.gpioId, PI_LOW) == 0)
    {
        res.hasSucceeded = true;
        ROS_INFO("Set GPIO %u to low level.", req.gpioId);
    }
    else
    {
        res.hasSucceeded = false;
        ROS_ERROR("Failed to set GPIO %u to low level!", req.gpioId);
    }
    return true;
}

bool PigpioOutput::sendTriggerPulse(hal_pigpio::hal_pigpioSendTriggerPulse::Request &req,
                                    hal_pigpio::hal_pigpioSendTriggerPulse::Response &res)
{
    int error = gpio_trigger(pigpioHandle, req.gpioId, req.pulseLengthInUs, PI_HIGH);
    if (error == 0)
    {
        res.hasSucceeded = true;
        ROS_INFO("Sent trigger pulse for GPIO %u.", req.gpioId);
    }
    else
    {
        res.hasSucceeded = false;
        ROS_ERROR("Failed to send trigger pulse for GPIO %u!", req.gpioId);
    }
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hal_pigpioOutput");
    ros::NodeHandle node;
    int pigpioHandle;

    ros::ServiceClient getPigpioHandle = node.serviceClient<hal_pigpio::hal_pigpioGetHandle>("hal_pigpioGetHandle");
    hal_pigpio::hal_pigpioGetHandle pigpioHandleRequest;
    getPigpioHandle.call(pigpioHandleRequest);
    pigpioHandle = pigpioHandleRequest.response.handle;

    PigpioOutput pigpioOutput(&node, pigpioHandle);

    ros::spin();

    return 0;
}