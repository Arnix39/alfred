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

void PigpioOutput::setPwmDutycycle(hal_pigpio::hal_pigpioSetPwmDutycycle::Request request,
                                   hal_pigpio::hal_pigpioSetPwmDutycycle::Response response)
{
    if (set_PWM_dutycycle(pigpioHandle, request->gpioId, request->dutycycle) == 0)
    {
        response->has_succeeded = true;
    }
    else
    {
        response->has_succeeded = false;
        RCLCPP_ERROR(halPigpioNode->get_logger(),"Failed to set PWM duty cycle for GPIO %u!", request->gpioId);
    }
    return true;
}

void PigpioOutput::setPwmFrequency(hal_pigpio::hal_pigpioSetPwmFrequency::Request request,
                                   hal_pigpio::hal_pigpioSetPwmFrequency::Response response)
{
    int pwmSettingResult = set_PWM_frequency(pigpioHandle, request->gpioId, request->frequency);

    if ((pwmSettingResult != PI_NOT_PERMITTED) && (pwmSettingResult != PI_BAD_USER_GPIO))
    {
        response->has_succeeded = true;
        RCLCPP_INFO(halPigpioNode->get_logger(),"Set PWM frequency of %u for GPIO %u.", request->frequency, request->gpioId);
    }
    else
    {
        response->has_succeeded = false;
        RCLCPP_ERROR(halPigpioNode->get_logger(),"Failed to set PWM frequency for GPIO %u!", request->gpioId);
    }
    return true;
}

void PigpioOutput::setGpioHigh(hal_pigpio::hal_pigpioSetGpioHigh::Request request,
                               hal_pigpio::hal_pigpioSetGpioHigh::Response response)
{
    if (gpio_write(pigpioHandle, request->gpioId, PI_HIGH) == 0)
    {
        response->has_succeeded = true;
    }
    else
    {
        response->has_succeeded = false;
        RCLCPP_ERROR(halPigpioNode->get_logger(),"Failed to set GPIO %u to high level!", request->gpioId);
    }
    return true;
}

void PigpioOutput::setGpioLow(hal_pigpio::hal_pigpioSetGpioLow::Request request,
                              hal_pigpio::hal_pigpioSetGpioLow::Response response)
{
    if (gpio_write(pigpioHandle, request->gpioId, PI_LOW) == 0)
    {
        response->has_succeeded = true;
    }
    else
    {
        response->has_succeeded = false;
        RCLCPP_ERROR(halPigpioNode->get_logger(),"Failed to set GPIO %u to low level!", request->gpioId);
    }
    return true;
}

void PigpioOutput::sendTriggerPulse(hal_pigpio::hal_pigpioSendTriggerPulse::Request request,
                                    hal_pigpio::hal_pigpioSendTriggerPulse::Response response)
{
    if (gpio_trigger(pigpioHandle, request->gpioId, request->pulseLengthInUs, PI_HIGH) == 0)
    {
        response->has_succeeded = true;
    }
    else
    {
        response->has_succeeded = false;
        RCLCPP_ERROR(halPigpioNode->get_logger(),"Failed to send trigger pulse for GPIO %u!", request->gpioId);
    }
    return true;
}