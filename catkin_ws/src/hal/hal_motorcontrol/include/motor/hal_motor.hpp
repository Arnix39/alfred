#ifndef HAL_MOTOR
#define HAL_MOTOR

#include "ros/ros.h"
#include "commonDefinitions.hpp"

// Services and messages headers (generated)
#include "hal_pigpio/hal_pigpioSetInputMode.h"
#include "hal_pigpio/hal_pigpioSetOutputMode.h"
#include "hal_pigpio/hal_pigpioSetEncoderCallback.h"
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
    ros::ServiceClient *setPwmDutycycleClientHandle;
    uint8_t id;

public:
    Motor(uint8_t gpioPwmChannelA, uint8_t gpioPwmChannelB, uint8_t gpioEncoderChannelA, uint8_t gpioEncoderChannelB, uint8_t motorId);
    ~Motor() = default;
    uint32_t getEncoderCount(void);
    void setEncoderCount(uint32_t count);
    void setPwmDutyCycle(uint16_t dutycycle, bool isDirectionForward);
    void configureGpios(ros::ServiceClient *setOutputClientHandle, ros::ServiceClient *setInputClientHandle, 
                        ros::ServiceClient *SetCallbackClientHandle, ros::ServiceClient *SetPwmFrequencyClientHandle,
                        uint8_t motorId);
    void configureSetPwmDutycycleClientHandle(ros::ServiceClient *setPwmDutycycleClient);
    uint8_t getId(void);
};

#endif