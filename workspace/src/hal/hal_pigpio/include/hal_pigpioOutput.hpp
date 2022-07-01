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
    rclcpp::ServiceServer setPwmDutycycleService;
    rclcpp::ServiceServer setPwmFrequencyService;
    rclcpp::ServiceServer setGpioHighService;
    rclcpp::ServiceServer setGpioLowService;
    rclcpp::ServiceServer sendTriggerPulseService;
    rclcpp::ServiceClient getModeClient;
    int pigpioHandle;
    std::shared_ptr<rclcpp::Node> halPigpioNode;

public:
    PigpioOutput(std::shared_ptr<rclcpp::Node> node, int pigpioHandle);
    ~PigpioOutput() = default;
    bool setPwmDutycycle(hal_pigpio::hal_pigpioSetPwmDutycycle::Request &req,
                         hal_pigpio::hal_pigpioSetPwmDutycycle::Response &res);
    bool setPwmFrequency(hal_pigpio::hal_pigpioSetPwmFrequency::Request &req,
                         hal_pigpio::hal_pigpioSetPwmFrequency::Response &res);
    bool setGpioHigh(hal_pigpio::hal_pigpioSetGpioHigh::Request &req,
                     hal_pigpio::hal_pigpioSetGpioHigh::Response &res);
    bool setGpioLow(hal_pigpio::hal_pigpioSetGpioLow::Request &req,
                    hal_pigpio::hal_pigpioSetGpioLow::Response &res);
    bool sendTriggerPulse(hal_pigpio::hal_pigpioSendTriggerPulse::Request &req,
                          hal_pigpio::hal_pigpioSendTriggerPulse::Response &res);
};

#endif