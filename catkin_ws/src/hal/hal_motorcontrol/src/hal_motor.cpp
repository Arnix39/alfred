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

void Motor::configureGpios(ros::ServiceClient *setOutputClientHandle, ros::ServiceClient *setInputClientHandle, 
                           ros::ServiceClient *setCallbackClientHandle, ros::ServiceClient *setPwmFrequencyClientHandle,
                           uint8_t motorId)
{
    hal_pigpio::hal_pigpioSetInputMode setInputModeSrv;
    hal_pigpio::hal_pigpioSetEncoderCallback setEncoderCallbackSrv;
    hal_pigpio::hal_pigpioSetOutputMode setOutputModeSrv;
    hal_pigpio::hal_pigpioSetPwmFrequency setPwmFrequencySrv;

    /* Encoder channel A */
    setInputModeSrv.request.gpioId = encoder.channelA.gpio;
    setInputClientHandle->call(setInputModeSrv);

    setEncoderCallbackSrv.request.gpioId = encoder.channelA.gpio;
    setEncoderCallbackSrv.request.edgeChangeType = AS_EITHER_EDGE;
    setEncoderCallbackSrv.request.motorId = id;
    setCallbackClientHandle->call(setEncoderCallbackSrv);

    /* Encoder channel B */
    setInputModeSrv.request.gpioId = encoder.channelB.gpio;
    setInputClientHandle->call(setInputModeSrv);

    setEncoderCallbackSrv.request.gpioId = encoder.channelB.gpio;
    setEncoderCallbackSrv.request.edgeChangeType = AS_EITHER_EDGE;
    setEncoderCallbackSrv.request.motorId = id;
    setCallbackClientHandle->call(setEncoderCallbackSrv);

    /* PWM channel A */
    setOutputModeSrv.request.gpioId = pwmA.gpio;
    setOutputClientHandle->call(setOutputModeSrv);

    setPwmFrequencySrv.request.gpioId = pwmA.gpio;
    setPwmFrequencySrv.request.frequency = MOTOR_PWM_FREQUENCY;
    setPwmFrequencyClientHandle->call(setPwmFrequencySrv);

    /* PWM channel B */
    setOutputModeSrv.request.gpioId = pwmB.gpio;
    setOutputClientHandle->call(setOutputModeSrv);

    setPwmFrequencySrv.request.gpioId = pwmB.gpio;
    setPwmFrequencySrv.request.frequency = MOTOR_PWM_FREQUENCY;
    setPwmFrequencyClientHandle->call(setPwmFrequencySrv);
}

void Motor::configureSetPwmDutycycleClientHandle(ros::ServiceClient *setPwmDutycycleClient)
{
    setPwmDutycycleClientHandle = setPwmDutycycleClient;
}

void Motor::configureSetMotorDirectionPublisherHandle(ros::Publisher *setMotorDirectionPublisher)
{
    setMotorDirectionPublisherHandle = setMotorDirectionPublisher;
}

uint32_t Motor::getEncoderCount(void)
{
    return encoder.encoderCount;
}

void Motor::setEncoderCount(uint32_t count)
{
    encoder.encoderCount = count;
}

void Motor::setPwmDutyCycleAndDirection(uint16_t dutycycle, bool isDirectionForward)
{
    hal_pigpio::hal_pigpioSetPwmDutycycle setPwmDutycycleSrv;
    hal_pigpio::hal_pigpioMotorDirectionMsg motorDirectionMsg;

    motorDirectionMsg.isDirectionForward = isDirectionForward;
    motorDirectionMsg.motorId = id;
    setMotorDirectionPublisherHandle->publish(motorDirectionMsg);

    pwmB.dutycycle = dutycycle;
    pwmA.dutycycle = dutycycle;

    if (isDirectionForward)
    {
        setPwmDutycycleSrv.request.gpioId = pwmA.gpio;
        setPwmDutycycleSrv.request.dutycycle = pwmA.dutycycle;
        setPwmDutycycleClientHandle->call(setPwmDutycycleSrv);

        setPwmDutycycleSrv.request.gpioId = pwmB.gpio;
        setPwmDutycycleSrv.request.dutycycle = 0;
        setPwmDutycycleClientHandle->call(setPwmDutycycleSrv);
    }
    else
    {
        setPwmDutycycleSrv.request.gpioId = pwmB.gpio;
        setPwmDutycycleSrv.request.dutycycle = pwmB.dutycycle;
        setPwmDutycycleClientHandle->call(setPwmDutycycleSrv);

        setPwmDutycycleSrv.request.gpioId = pwmA.gpio;
        setPwmDutycycleSrv.request.dutycycle = 0;
        setPwmDutycycleClientHandle->call(setPwmDutycycleSrv);
    }
}

uint8_t Motor::getId(void)
{
    return id;
}