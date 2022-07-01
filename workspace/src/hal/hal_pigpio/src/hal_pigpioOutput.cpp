#include "hal_pigpioOutput.hpp"

PigpioOutput::PigpioOutput(std::shared_ptr<rclcpp::Node> node, int pigpioHandle) :  pigpioHandle(pigpioHandle),
                                                                                    halPigpioNode(node),
                                                                                    getModeClient(node->serviceClient<hal_pigpio::hal_pigpioGetMode>("hal_pigpioGetMode")),
                                                                                    setPwmDutycycleService(node->advertiseService("hal_pigpioSetPwmDutycycle", &PigpioOutput::setPwmDutycycle, this)),
                                                                                    setPwmFrequencyService(node->advertiseService("hal_pigpioSetPwmFrequency", &PigpioOutput::setPwmFrequency, this)),
                                                                                    setGpioHighService(node->advertiseService("hal_pigpioSetGpioHigh", &PigpioOutput::setGpioHigh, this)),
                                                                                    setGpioLowService(node->advertiseService("hal_pigpioSetGpioLow", &PigpioOutput::setGpioLow, this)),
                                                                                    sendTriggerPulseService(node->advertiseService("hal_pigpioSendTriggerPulse", &PigpioOutput::sendTriggerPulse, this))
{
}

bool PigpioOutput::setPwmDutycycle(hal_pigpio::hal_pigpioSetPwmDutycycle::Request &req,
                                   hal_pigpio::hal_pigpioSetPwmDutycycle::Response &res)
{
    if (set_PWM_dutycycle(pigpioHandle, req.gpioId, req.dutycycle) == 0)
    {
        res.hasSucceeded = true;
    }
    else
    {
        res.hasSucceeded = false;
        RCLCPP_ERROR(halPigpioNode->get_logger(),"Failed to set PWM duty cycle for GPIO %u!", req.gpioId);
    }
    return true;
}

bool PigpioOutput::setPwmFrequency(hal_pigpio::hal_pigpioSetPwmFrequency::Request &req,
                                   hal_pigpio::hal_pigpioSetPwmFrequency::Response &res)
{
    int pwmSettingResult = set_PWM_frequency(pigpioHandle, req.gpioId, req.frequency);

    if ((pwmSettingResult != PI_NOT_PERMITTED) && (pwmSettingResult != PI_BAD_USER_GPIO))
    {
        res.hasSucceeded = true;
        RCLCPP_INFO(halPigpioNode->get_logger(),"Set PWM frequency of %u for GPIO %u.", req.frequency, req.gpioId);
    }
    else
    {
        res.hasSucceeded = false;
        RCLCPP_ERROR(halPigpioNode->get_logger(),"Failed to set PWM frequency for GPIO %u!", req.gpioId);
    }
    return true;
}

bool PigpioOutput::setGpioHigh(hal_pigpio::hal_pigpioSetGpioHigh::Request &req,
                               hal_pigpio::hal_pigpioSetGpioHigh::Response &res)
{
    if (gpio_write(pigpioHandle, req.gpioId, PI_HIGH) == 0)
    {
        res.hasSucceeded = true;
    }
    else
    {
        res.hasSucceeded = false;
        RCLCPP_ERROR(halPigpioNode->get_logger(),"Failed to set GPIO %u to high level!", req.gpioId);
    }
    return true;
}

bool PigpioOutput::setGpioLow(hal_pigpio::hal_pigpioSetGpioLow::Request &req,
                              hal_pigpio::hal_pigpioSetGpioLow::Response &res)
{
    if (gpio_write(pigpioHandle, req.gpioId, PI_LOW) == 0)
    {
        res.hasSucceeded = true;
    }
    else
    {
        res.hasSucceeded = false;
        RCLCPP_ERROR(halPigpioNode->get_logger(),"Failed to set GPIO %u to low level!", req.gpioId);
    }
    return true;
}

bool PigpioOutput::sendTriggerPulse(hal_pigpio::hal_pigpioSendTriggerPulse::Request &req,
                                    hal_pigpio::hal_pigpioSendTriggerPulse::Response &res)
{
    if (gpio_trigger(pigpioHandle, req.gpioId, req.pulseLengthInUs, PI_HIGH) == 0)
    {
        res.hasSucceeded = true;
    }
    else
    {
        res.hasSucceeded = false;
        RCLCPP_ERROR(halPigpioNode->get_logger(),"Failed to send trigger pulse for GPIO %u!", req.gpioId);
    }
    return true;
}