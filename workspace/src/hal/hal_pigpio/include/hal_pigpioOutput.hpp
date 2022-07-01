#ifndef HAL_PIGPIO_OUTPUT
#define HAL_PIGPIO_OUTPUT

#include "rclcpp/rclcpp.hpp"

// Pigpio library
#include <pigpiod_if2.h>

// Services and messages headers (generated)
#include "hal_pigpio/srv/hal_pigpio_set_pwm_dutycycle.hpp"
#include "hal_pigpio/srv/hal_pigpio_set_pwm_frequency.hpp"
#include "hal_pigpio/srv/hal_pigpio_set_gpio_high.hpp"
#include "hal_pigpio/srv/hal_pigpio_set_gpio_low.hpp"
#include "hal_pigpio/srv/hal_pigpio_send_trigger_pulse.hpp"
#include "hal_pigpio/srv/hal_pigpio_get_mode.hpp"

class PigpioOutput
{
private:
    int pigpioHandle;
    std::shared_ptr<rclcpp::Node> halPigpioNode;
    rclcpp::ServiceServer setPwmDutycycleService;
    rclcpp::ServiceServer setPwmFrequencyService;
    rclcpp::ServiceServer setGpioHighService;
    rclcpp::ServiceServer setGpioLowService;
    rclcpp::ServiceServer sendTriggerPulseService;
    rclcpp::ServiceClient getModeClient;

public:
    PigpioOutput(std::shared_ptr<rclcpp::Node> node, int pigpioHandle);
    ~PigpioOutput() = default;
    bool setPwmDutycycle(hal_pigpio::hal_pigpioSetPwmDutycycle::Request request,
                         hal_pigpio::hal_pigpioSetPwmDutycycle::Response response);
    bool setPwmFrequency(hal_pigpio::hal_pigpioSetPwmFrequency::Request request,
                         hal_pigpio::hal_pigpioSetPwmFrequency::Response response);
    bool setGpioHigh(hal_pigpio::hal_pigpioSetGpioHigh::Request request,
                     hal_pigpio::hal_pigpioSetGpioHigh::Response response);
    bool setGpioLow(hal_pigpio::hal_pigpioSetGpioLow::Request request,
                    hal_pigpio::hal_pigpioSetGpioLow::Response response);
    bool sendTriggerPulse(hal_pigpio::hal_pigpioSendTriggerPulse::Request request,
                          hal_pigpio::hal_pigpioSendTriggerPulse::Response response);
};

#endif