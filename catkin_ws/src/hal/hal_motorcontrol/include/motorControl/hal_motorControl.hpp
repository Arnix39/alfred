#ifndef HAL_MOTORCONTROL
#define HAL_MOTORCONTROL

#include "ros/ros.h"
#include "hal_motorControlVirtuals.hpp"
#include "commonDefinitions.hpp"

// Services and messages headers (generated)
#include "hal_pigpio/hal_pigpioSetInputMode.h"
#include "hal_pigpio/hal_pigpioSetOutputMode.h"
#include "hal_pigpio/hal_pigpioSetCallback.h"
#include "hal_pigpio/hal_pigpioSetPwmFrequency.h"
#include "hal_pigpio/hal_pigpioSetPwmDutycycle.h"
#include "hal_pigpio/hal_pigpioEdgeChangeMsg.h"
#include "hal_motorcontrol/hal_motorcontrolMsg.h"

class MotorControl
{
private:
    MotorControlPublisher *motorControlPub;
    MotorControlClients *motorControlClients;
    MotorControlSubscriber *motorControlSub;
    uint32_t rightEncoderCount;
    uint32_t leftEncoderCount;

public:
    MotorControl(MotorControlSubscriber *motorControlSubscriber, MotorControlPublisher *motorControlPub, MotorControlClients *motorControlServiceClients);
    ~MotorControl() = default;
    void publishMessage(void);
    void edgeChangeCallback(const hal_pigpio::hal_pigpioEdgeChangeMsg &msg);
};

#endif