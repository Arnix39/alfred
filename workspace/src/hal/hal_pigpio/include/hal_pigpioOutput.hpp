#ifndef HAL_PIGPIO_OUTPUT
#define HAL_PIGPIO_OUTPUT

#include "rclcpp/rclcpp.hpp"

// Pigpio library
#include <pigpiod_if2.h>

// Services and messages headers (generated)
#include "hal_pigpio_interfaces/srv/hal_pigpio_set_pwm_dutycycle.hpp"
#include "hal_pigpio_interfaces/srv/hal_pigpio_set_pwm_frequency.hpp"
#include "hal_pigpio_interfaces/srv/hal_pigpio_set_gpio_high.hpp"
#include "hal_pigpio_interfaces/srv/hal_pigpio_set_gpio_low.hpp"
#include "hal_pigpio_interfaces/srv/hal_pigpio_send_trigger_pulse.hpp"
#include "hal_pigpio_interfaces/srv/hal_pigpio_get_mode.hpp"

class PigpioOutput
{
private:
    int pigpioHandle;
    std::shared_ptr<rclcpp::Node> halPigpioNode;
    rclcpp::Client<hal_pigpio_interfaces::srv::HalPigpioGetMode>::SharedPtr getModeClient;
    rclcpp::Service<hal_pigpio_interfaces::srv::HalPigpioSetPwmDutycycle>::SharedPtr setPwmDutycycleService;
    rclcpp::Service<hal_pigpio_interfaces::srv::HalPigpioSetPwmFrequency>::SharedPtr setPwmFrequencyService;
    rclcpp::Service<hal_pigpio_interfaces::srv::HalPigpioSetGpioHigh>::SharedPtr setGpioHighService;
    rclcpp::Service<hal_pigpio_interfaces::srv::HalPigpioSetGpioLow>::SharedPtr setGpioLowService;
    rclcpp::Service<hal_pigpio_interfaces::srv::HalPigpioSendTriggerPulse>::SharedPtr sendTriggerPulseService;

public:
    PigpioOutput(std::shared_ptr<rclcpp::Node> node, int pigpioHandle);
    ~PigpioOutput() = default;
    void setPwmDutycycle(const std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioSetPwmDutycycle::Request> request,
                         std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioSetPwmDutycycle::Response> response);
    void setPwmFrequency(const std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioSetPwmFrequency::Request> request,
                         std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioSetPwmFrequency::Response> response);
    void setGpioHigh(const std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioSetGpioHigh::Request> request,
                    std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioSetGpioHigh::Response> response);
    void setGpioLow(const std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioSetGpioLow::Request> request,
                    std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioSetGpioLow::Response> response);
    void sendTriggerPulse(const std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioSendTriggerPulse::Request> request,
                          std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioSendTriggerPulse::Response> response);
};

#endif