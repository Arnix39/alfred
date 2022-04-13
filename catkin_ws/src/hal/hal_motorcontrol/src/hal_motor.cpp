#include "hal_motor.hpp"

Motor::Motor(uint8_t gpioPwmChannelA, uint8_t gpioPwmChannelB,
             uint8_t gpioEncoderChannelA, uint8_t gpioEncoderChannelB) : encoder{.channelA{.gpio = gpioEncoderChannelA},
                                                                                 .channelB{.gpio = gpioEncoderChannelB},
                                                                                 .encoderCount = 0},
                                                                         pwmA{.gpio=gpioPwmChannelA, .dutycycle = 0},
                                                                         pwmB{.gpio=gpioPwmChannelB, .dutycycle = 0},
                                                                         isDirectionForward(true)
{
};

void Motor::configure(ros::ServiceClient *setOutputClientHandle, ros::ServiceClient *setInputClientHandle, 
                      ros::ServiceClient *SetCallbackClientHandle, 
                      ros::ServiceClient *SetPwmFrequencyClientHandle, ros::ServiceClient *SetPwmDutycycleClientHandle)
{
    hal_pigpio::hal_pigpioSetInputMode setInputModeSrv;
    hal_pigpio::hal_pigpioSetCallback setCallbackSrv;
    hal_pigpio::hal_pigpioSetOutputMode setOutputModeSrv;
    hal_pigpio::hal_pigpioSetPwmFrequency setPwmFrequencySrv;

    /* Encoder channel A */
    setInputModeSrv.request.gpioId = encoder.channelA.gpio;
    setInputClientHandle->call(setInputModeSrv);

    setCallbackSrv.request.gpioId = encoder.channelA.gpio;
    setCallbackSrv.request.edgeChangeType = AS_EITHER_EDGE;
    SetCallbackClientHandle->call(setCallbackSrv);

    /* Encoder channel B */
    setInputModeSrv.request.gpioId = encoder.channelB.gpio;
    setInputClientHandle->call(setInputModeSrv);

    setCallbackSrv.request.gpioId = encoder.channelB.gpio;
    setCallbackSrv.request.edgeChangeType = AS_EITHER_EDGE;
    SetCallbackClientHandle->call(setCallbackSrv);

    /* PWM channel A */
    setOutputModeSrv.request.gpioId = pwmA.gpio;
    setOutputClientHandle->call(setOutputModeSrv);

    setPwmFrequencySrv.request.gpioId = pwmA.gpio;
    setPwmFrequencySrv.request.frequency = MOTOR_PWM_FREQUENCY;
    SetPwmFrequencyClientHandle->call(setPwmFrequencySrv);

    /* PWM channel B */
    setOutputModeSrv.request.gpioId = pwmB.gpio;
    setOutputClientHandle->call(setOutputModeSrv);

    setPwmFrequencySrv.request.gpioId = pwmB.gpio;
    setPwmFrequencySrv.request.frequency = MOTOR_PWM_FREQUENCY;
    SetPwmFrequencyClientHandle->call(setPwmFrequencySrv);
}

void Motor::edgeChangeCallback(const hal_pigpio::hal_pigpioEdgeChangeMsg &msg)
{
    if (msg.gpioId == encoder.channelA.gpio || msg.gpioId == encoder.channelB.gpio)
    {
        if(isDirectionForward)
        {
            incrementEncoderCount();
        }
        else
        {
            decrementEncoderCount();
        }
    }
}

uint32_t Motor::getEncoderCount(void)
{
    return encoder.encoderCount;
}

void Motor::incrementEncoderCount(void)
{
    ++encoder.encoderCount;
}

void Motor::decrementEncoderCount(void)
{
    --encoder.encoderCount;
}

void Motor::resetEncoderCount(void)
{
    encoder.encoderCount = 0;
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