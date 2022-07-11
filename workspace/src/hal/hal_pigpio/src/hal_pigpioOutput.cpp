#include "hal_pigpioOutput.hpp"

using namespace std::placeholders;

PigpioOutput::PigpioOutput(std::shared_ptr<rclcpp::Node> node, int pigpioHandle) :  pigpioHandle(pigpioHandle),
                                                                                    halPigpioNode(node),
                                                                                    getModeClient(node->create_client<hal_pigpio_interfaces::srv::HalPigpioGetMode>("hal_pigpioGetMode")),
                                                                                    setPwmDutycycleService(node->create_service<hal_pigpio_interfaces::srv::HalPigpioSetPwmDutycycle>("hal_pigpioSetPwmDutycycle", std::bind(&PigpioOutput::setPwmDutycycle, this, _1, _2))),
                                                                                    setPwmFrequencyService(node->create_service<hal_pigpio_interfaces::srv::HalPigpioSetPwmFrequency>("hal_pigpioSetPwmFrequency", std::bind(&PigpioOutput::setPwmFrequency, this, _1, _2))),
                                                                                    setGpioHighService(node->create_service<hal_pigpio_interfaces::srv::HalPigpioSetGpioHigh>("hal_pigpioSetGpioHigh", std::bind(&PigpioOutput::setGpioHigh, this, _1, _2))),
                                                                                    setGpioLowService(node->create_service<hal_pigpio_interfaces::srv::HalPigpioSetGpioLow>("hal_pigpioSetGpioLow", std::bind(&PigpioOutput::setGpioLow, this, _1, _2))),
                                                                                    sendTriggerPulseService(node->create_service<hal_pigpio_interfaces::srv::HalPigpioSendTriggerPulse>("hal_pigpioSendTriggerPulse", std::bind(&PigpioOutput::sendTriggerPulse, this, _1, _2)))
{
}

void PigpioOutput::setPwmDutycycle(const std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioSetPwmDutycycle::Request> request,
                     std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioSetPwmDutycycle::Response> response)
{
    if (set_PWM_dutycycle(pigpioHandle, request->gpio_id, request->dutycycle) == 0)
    {
        response->has_succeeded = true;
    }
    else
    {
        response->has_succeeded = false;
        RCLCPP_ERROR(halPigpioNode->get_logger(),"Failed to set PWM duty cycle for GPIO %u!", request->gpio_id);
    }
}

void PigpioOutput::setPwmFrequency(const std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioSetPwmFrequency::Request> request,
                                   std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioSetPwmFrequency::Response> response)
{
    int pwmSettingResult = set_PWM_frequency(pigpioHandle, request->gpio_id, request->frequency);

    if ((pwmSettingResult != PI_NOT_PERMITTED) && (pwmSettingResult != PI_BAD_USER_GPIO))
    {
        response->has_succeeded = true;
        RCLCPP_INFO(halPigpioNode->get_logger(),"Set PWM frequency of %u for GPIO %u.", request->frequency, request->gpio_id);
    }
    else
    {
        response->has_succeeded = false;
        RCLCPP_ERROR(halPigpioNode->get_logger(),"Failed to set PWM frequency for GPIO %u!", request->gpio_id);
    }
}

void PigpioOutput::setGpioHigh(const std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioSetGpioHigh::Request> request,
                               std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioSetGpioHigh::Response> response)
{
    if (gpio_write(pigpioHandle, request->gpio_id, PI_HIGH) == 0)
    {
        response->has_succeeded = true;
    }
    else
    {
        response->has_succeeded = false;
        RCLCPP_ERROR(halPigpioNode->get_logger(),"Failed to set GPIO %u to high level!", request->gpio_id);
    }
}

void PigpioOutput::setGpioLow(const std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioSetGpioLow::Request> request,
                              std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioSetGpioLow::Response> response)
{
    if (gpio_write(pigpioHandle, request->gpio_id, PI_LOW) == 0)
    {
        response->has_succeeded = true;
    }
    else
    {
        response->has_succeeded = false;
        RCLCPP_ERROR(halPigpioNode->get_logger(),"Failed to set GPIO %u to low level!", request->gpio_id);
    }
}

void PigpioOutput::sendTriggerPulse(const std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioSendTriggerPulse::Request> request,
                                    std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioSendTriggerPulse::Response> response)
{
    if (gpio_trigger(pigpioHandle, request->gpio_id, request->pulse_length_in_us, PI_HIGH) == 0)
    {
        response->has_succeeded = true;
    }
    else
    {
        response->has_succeeded = false;
        RCLCPP_ERROR(halPigpioNode->get_logger(),"Failed to send trigger pulse for GPIO %u!", request->gpio_id);
    }
}