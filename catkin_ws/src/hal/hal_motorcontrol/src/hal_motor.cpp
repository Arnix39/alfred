#include "hal_motor.hpp"

Motor::Motor(uint8_t gpioPwmChannelA, uint8_t gpioPwmChannelB,
             uint8_t gpioEncoderChannelA, uint8_t gpioEncoderChannelB) : encoder{.channelA{.gpio=gpioEncoderChannelA},
                                                                                 .channelB{.gpio=gpioEncoderChannelB},
                                                                                 .encoderCounts=0},
                                                                         pwmA{.gpio=gpioPwmChannelA, .dutycycle=0},
                                                                         pwmB{.gpio=gpioPwmChannelB, .dutycycle=0},
                                                                         isDirectionForward(true)
{
};

void Motor::configureGpios(ros::ServiceClient *setOutputClientHandle, ros::ServiceClient *setInputClientHandle, ros::ServiceClient *SetCallbackClientHandle)
{
    hal_pigpio::hal_pigpioSetInputMode setInputModeSrv;
    hal_pigpio::hal_pigpioSetCallback setCallbackSrv;
    hal_pigpio::hal_pigpioSetOutputMode setOutputModeSrv;

    /* Encoder channel A */
    setInputModeSrv.request.gpioId = encoder.channelA.gpio;
    setInputClientHandle->call(setInputModeSrv);

    setCallbackSrv.request.gpioId = encoder.channelA.gpio;
    setCallbackSrv.request.edgeChangeType = AS_RISING_EDGE;
    SetCallbackClientHandle->call(setCallbackSrv);

    /* Encoder channel B */
    setInputModeSrv.request.gpioId = encoder.channelB.gpio;
    setInputClientHandle->call(setInputModeSrv);

    setCallbackSrv.request.gpioId = encoder.channelB.gpio;
    setCallbackSrv.request.edgeChangeType = AS_RISING_EDGE;
    SetCallbackClientHandle->call(setCallbackSrv);

    /* PWM channel A */
    setOutputModeSrv.request.gpioId = pwmA.gpio;
    setOutputClientHandle->call(setOutputModeSrv);

    /* PWM channel B */
    setOutputModeSrv.request.gpioId = pwmB.gpio;
    setOutputClientHandle->call(setOutputModeSrv);
}

uint32_t Motor::getEncoderCounts(void)
{
    return encoder.encoderCounts;
}

void Motor::setPwmDutyCycle(Channel channel, uint16_t dutycycle)
{
    if (channel == chA)
    {
        pwmA.dutycycle = dutycycle;
    }
    else if (channel == chB)
    {
        pwmB.dutycycle = dutycycle;
    }
}

void Motor::setDirectionForward(void)
{
    isDirectionForward = true;
}

void Motor::setDirectionBackward(void)
{
    isDirectionForward = false;
}