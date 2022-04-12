#ifndef HAL_MOTOR
#define HAL_MOTOR

#include "ros/ros.h"
//#include "hal_motorVirtuals.hpp"
#include "commonDefinitions.hpp"

// Services and messages headers (generated)
#include "hal_pigpio/hal_pigpioSetInputMode.h"
#include "hal_pigpio/hal_pigpioSetOutputMode.h"
#include "hal_pigpio/hal_pigpioSetCallback.h"
#include "hal_pigpio/hal_pigpioSetPwmFrequency.h"
#include "hal_pigpio/hal_pigpioSetPwmDutycycle.h"
#include "hal_pigpio/hal_pigpioEdgeChangeMsg.h"

struct EncoderChannel
{
    uint8_t gpio;
};

struct Encoder
{
    EncoderChannel channelA;
    EncoderChannel channelB;
    uint32_t encoderCounts;
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
    bool isDirectionForward;

public:
    Motor(uint8_t gpioPwmChannelA, uint8_t gpioPwmChannelB, uint8_t gpioEncoderChannelA, uint8_t gpioEncoderChannelB);
    ~Motor() = default;
    uint32_t getEncoderCounts(void);
    void setPwmDutyCycle(Channel channel, uint16_t dutycycle);
    void setDirectionForward(void);
    void setDirectionBackward(void);
};

#endif