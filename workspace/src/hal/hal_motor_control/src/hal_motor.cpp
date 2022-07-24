#include "hal_motor.hpp"

Motor::Motor(uint8_t gpioPwmChannelA, uint8_t gpioPwmChannelB, 
             uint8_t gpioEncoderChannelA, uint8_t gpioEncoderChannelB,
             uint8_t motorId) : encoder{.channelA{.gpio = gpioEncoderChannelA},
                                        .channelB{.gpio = gpioEncoderChannelB},
                                        .encoderCount = 0},
                                pwmA{.gpio=gpioPwmChannelA, .dutycycle = 0},
                                pwmB{.gpio=gpioPwmChannelB, .dutycycle = 0},
                                id(motorId)
{
};

void Motor::configureGpios(rclcpp::Client<hal_pigpio_interfaces::srv::HalPigpioSetOutputMode>::SharedPtr gpioSetOutputModeClient, 
                           rclcpp::Client<hal_pigpio_interfaces::srv::HalPigpioSetInputMode>::SharedPtr gpioSetInputModeClient, 
                           rclcpp::Client<hal_pigpio_interfaces::srv::HalPigpioSetEncoderCallback>::SharedPtr gpioSetEncoderCallbackClient, 
                           rclcpp::Client<hal_pigpio_interfaces::srv::HalPigpioSetPwmFrequency>::SharedPtr gpioSetPwmFrequencyClient)
{
    auto setInputModeEncoderChARequest = std::make_shared<hal_pigpio_interfaces::srv::HalPigpioSetInputMode::Request>();
    auto setInputModeEncoderChBRequest = std::make_shared<hal_pigpio_interfaces::srv::HalPigpioSetInputMode::Request>();
    auto setEncoderCallbackChARequest = std::make_shared<hal_pigpio_interfaces::srv::HalPigpioSetEncoderCallback::Request>();
    auto setEncoderCallbackChBRequest = std::make_shared<hal_pigpio_interfaces::srv::HalPigpioSetEncoderCallback::Request>();
    auto setOutputModePwmChARequest = std::make_shared<hal_pigpio_interfaces::srv::HalPigpioSetOutputMode::Request>();
    auto setOutputModePwmChBRequest = std::make_shared<hal_pigpio_interfaces::srv::HalPigpioSetOutputMode::Request>();
    auto setPwmFrequencyPwmChARequest = std::make_shared<hal_pigpio_interfaces::srv::HalPigpioSetPwmFrequency::Request>();
    auto setPwmFrequencyPwmChBRequest = std::make_shared<hal_pigpio_interfaces::srv::HalPigpioSetPwmFrequency::Request>();
    
    auto setInputModeCallback = [this](SetInputModeFuture_t future) 
    {
        if(!future.get()->has_succeeded)
        {
            RCLCPP_ERROR(rclcpp::get_logger("Motors"), "Failed to call service setInputMode");
        }   
    };

    auto setEncoderCallbackCallback = [this](SetEncoderCallbackFuture_t future) 
    {
        if(!future.get()->has_succeeded)
        {
            RCLCPP_ERROR(rclcpp::get_logger("Motors"), "Failed to call service setEncoderCallback");
        }   
    };

    auto setOutputModeCallback = [this](SetOutputModeFuture_t future) 
    {
        if(!future.get()->has_succeeded)
        {
            RCLCPP_ERROR(rclcpp::get_logger("Motors"), "Failed to call service setOutputMode");
        }   
    };

    auto setPwmFrequencyCallback = [this](SetPwmFrequencyFuture_t future) 
    {
        if(!future.get()->has_succeeded)
        {
            RCLCPP_ERROR(rclcpp::get_logger("Motors"), "Failed to call service setPwmFrequency");
        }   
    };

    /* Encoder channel A */
    setInputModeEncoderChARequest->gpio_id = encoder.channelA.gpio;
    auto setInputModeEncoderChAFuture = gpioSetInputModeClient->async_send_request(setInputModeEncoderChARequest, setInputModeCallback);

    setEncoderCallbackChARequest->gpio_id = encoder.channelA.gpio;
    setEncoderCallbackChARequest->edge_change_type = AS_EITHER_EDGE;
    setEncoderCallbackChARequest->motor_id = id;
    auto setEncoderCallbackChAFuture = gpioSetEncoderCallbackClient->async_send_request(setEncoderCallbackChARequest, setEncoderCallbackCallback);

    /* Encoder channel B */
    setInputModeEncoderChBRequest->gpio_id = encoder.channelB.gpio;
    auto setInputModeEncoderChBFuture = gpioSetInputModeClient->async_send_request(setInputModeEncoderChBRequest, setInputModeCallback);

    setEncoderCallbackChBRequest->gpio_id = encoder.channelB.gpio;
    setEncoderCallbackChBRequest->edge_change_type = AS_EITHER_EDGE;
    setEncoderCallbackChBRequest->motor_id = id;
    auto setEncoderCallbackChBFuture = gpioSetEncoderCallbackClient->async_send_request(setEncoderCallbackChBRequest, setEncoderCallbackCallback);

    /* PWM channel A */
    setOutputModePwmChARequest->gpio_id = pwmA.gpio;
    auto setOutputModePwmChAFuture = gpioSetOutputModeClient->async_send_request(setOutputModePwmChARequest, setOutputModeCallback);

    setPwmFrequencyPwmChARequest->gpio_id = pwmA.gpio;
    setPwmFrequencyPwmChARequest->frequency = MOTOR_PWM_FREQUENCY;
    auto setPwmFrequencyPwmChAFuture = gpioSetPwmFrequencyClient->async_send_request(setPwmFrequencyPwmChARequest, setPwmFrequencyCallback);


    /* PWM channel B */
    setOutputModePwmChBRequest->gpio_id = pwmB.gpio;
    auto setOutputModePwmChBFuture = gpioSetOutputModeClient->async_send_request(setOutputModePwmChBRequest, setOutputModeCallback);

    setPwmFrequencyPwmChBRequest->gpio_id = pwmB.gpio;
    setPwmFrequencyPwmChBRequest->frequency = MOTOR_PWM_FREQUENCY;
    auto setPwmFrequencyPwmChBFuture = gpioSetPwmFrequencyClient->async_send_request(setPwmFrequencyPwmChBRequest, setPwmFrequencyCallback);
}

uint32_t Motor::getEncoderCount(void)
{
    return encoder.encoderCount;
}

void Motor::setEncoderCount(uint32_t count)
{
    encoder.encoderCount = count;
}

void Motor::setPwmDutyCycleAndDirection(rclcpp::Client<hal_pigpio_interfaces::srv::HalPigpioSetPwmDutycycle>::SharedPtr gpioSetPwmDutycycleClient, 
                                        uint16_t dutycycle, 
                                        rclcpp::Client<hal_pigpio_interfaces::srv::HalPigpioSetMotorDirection>::SharedPtr gpioSetMotorDirectionClient, 
                                        bool isDirectionForward)
{
    auto setPwmDutycyclePwmChARequest = std::make_shared<hal_pigpio_interfaces::srv::HalPigpioSetPwmDutycycle::Request>();
    auto setPwmDutycyclePwmChBRequest = std::make_shared<hal_pigpio_interfaces::srv::HalPigpioSetPwmDutycycle::Request>();
    auto setMotorDirectionRequest = std::make_shared<hal_pigpio_interfaces::srv::HalPigpioSetMotorDirection::Request>();

    auto setPwmDutycycleCallback = [this](SetPwmDutycycleFuture_t future) 
    {
        if(!future.get()->has_succeeded)
        {
            RCLCPP_ERROR(rclcpp::get_logger("Motors"), "Failed to call service setPwmDutycycle");
        }   
    };

    auto setMotorDirectionCallback = [this](SetMotorDirectionFuture_t future) 
    {
        if(!future.get()->has_succeeded)
        {
            RCLCPP_ERROR(rclcpp::get_logger("Motors"), "Failed to call service setMotorDirection");
        }   
    };


    setMotorDirectionRequest->is_direction_forward = isDirectionForward;
    setMotorDirectionRequest->motor_id = id;
    auto setMotorDirectionFuture = gpioSetMotorDirectionClient->async_send_request(setMotorDirectionRequest, setMotorDirectionCallback);

    pwmB.dutycycle = dutycycle;
    pwmA.dutycycle = dutycycle;

    if (isDirectionForward)
    {
        setPwmDutycyclePwmChARequest->gpio_id = pwmA.gpio;
        setPwmDutycyclePwmChARequest->dutycycle = pwmA.dutycycle;
        auto setPwmDutycyclePwmChAFuture = gpioSetPwmDutycycleClient->async_send_request(setPwmDutycyclePwmChARequest, setPwmDutycycleCallback);

        setPwmDutycyclePwmChBRequest->gpio_id = pwmB.gpio;
        setPwmDutycyclePwmChBRequest->dutycycle = pwmB.dutycycle;
        auto setPwmDutycyclePwmChBFuture = gpioSetPwmDutycycleClient->async_send_request(setPwmDutycyclePwmChBRequest, setPwmDutycycleCallback);
    }
    else
    {
        setPwmDutycyclePwmChBRequest->gpio_id = pwmB.gpio;
        setPwmDutycyclePwmChBRequest->dutycycle = pwmB.dutycycle;
        auto setPwmDutycyclePwmChBFuture = gpioSetPwmDutycycleClient->async_send_request(setPwmDutycyclePwmChBRequest, setPwmDutycycleCallback);

        setPwmDutycyclePwmChARequest->gpio_id = pwmA.gpio;
        setPwmDutycyclePwmChARequest->dutycycle = 0;
        auto setPwmDutycyclePwmChAFuture = gpioSetPwmDutycycleClient->async_send_request(setPwmDutycyclePwmChARequest, setPwmDutycycleCallback);
    }
}

uint8_t Motor::getId(void)
{
    return id;
}