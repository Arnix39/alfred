#include "hal_motor.hpp"

Motor::Motor(uint8_t gpioPwmChannelA, uint8_t gpioPwmChannelB,\
             uint8_t gpioEncoderChannelA, uint8_t gpioEncoderChannelB) : encoder{.channelA{.gpio=gpioEncoderChannelA},\
                                                                                 .channelB{.gpio=gpioEncoderChannelB},\
                                                                                 .encoderCounts=0},
                                                                         pwmA{.gpio=gpioPwmChannelA, .dutycycle=0},
                                                                         pwmB{.gpio=gpioPwmChannelB, .dutycycle=0},
                                                                         isDirectionForward(true)
{
};

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