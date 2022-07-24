#ifndef HAL_MOTOR
#define HAL_MOTOR

#include "hal_motor_control_commonDefinitions.hpp"

struct EncoderChannel
{
    uint8_t gpio;
};

struct Encoder
{
    EncoderChannel channelA;
    EncoderChannel channelB;
    uint32_t encoderCount;
};

struct Pwm
{
    uint8_t gpio;
    uint16_t dutycycle;
};

class Motor
{
private:
    Encoder encoder;
    Pwm pwmA;
    Pwm pwmB;
    uint8_t id;

public:
    Motor(uint8_t gpioPwmChannelA, uint8_t gpioPwmChannelB, uint8_t gpioEncoderChannelA, uint8_t gpioEncoderChannelB, uint8_t motorId);
    ~Motor() = default;
    
    uint32_t getEncoderCount(void);
    void setEncoderCount(uint32_t count);
    void setPwmDutyCycleAndDirection(rclcpp::Client<hal_pigpio_interfaces::srv::HalPigpioSetPwmDutycycle>::SharedPtr gpioSetPwmDutycycleClient, 
                                     uint16_t dutycycle, 
                                     rclcpp::Client<hal_pigpio_interfaces::srv::HalPigpioSetMotorDirection>::SharedPtr gpioSetMotorDirectionClient, 
                                     bool isDirectionForward);
    void configureGpios(rclcpp::Client<hal_pigpio_interfaces::srv::HalPigpioSetOutputMode>::SharedPtr gpioSetOutputModeClient, 
                        rclcpp::Client<hal_pigpio_interfaces::srv::HalPigpioSetInputMode>::SharedPtr gpioSetInputModeClient, 
                        rclcpp::Client<hal_pigpio_interfaces::srv::HalPigpioSetEncoderCallback>::SharedPtr gpioSetEncoderCallbackClient, 
                        rclcpp::Client<hal_pigpio_interfaces::srv::HalPigpioSetPwmFrequency>::SharedPtr gpioSetPwmFrequencyClient);
    uint8_t getId(void);
};

#endif