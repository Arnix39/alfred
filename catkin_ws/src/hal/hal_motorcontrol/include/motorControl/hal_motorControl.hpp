#ifndef HAL_MOTOR_CONTROL
#define HAL_MOTOR_CONTROL

#include "ros/ros.h"
#include "hal_motorControlVirtuals.hpp"
#include "commonDefinitions.hpp"
#include "hal_motor.hpp"

// Services and messages headers (generated)
#include "hal_pigpio/hal_pigpioSetInputMode.h"
#include "hal_pigpio/hal_pigpioSetOutputMode.h"
#include "hal_pigpio/hal_pigpioSetCallback.h"
#include "hal_pigpio/hal_pigpioSetPwmFrequency.h"
#include "hal_pigpio/hal_pigpioSetPwmDutycycle.h"
#include "hal_pigpio/hal_pigpioEdgeChangeMsg.h"
#include "hal_pigpio/hal_pigpioHeartbeatMsg.h"
#include "hal_motorcontrol/hal_motorcontrolMsg.h"

class MotorControl
{
private:
    MotorControlPublisher *motorControlPub;
    MotorControlClients *motorControlClients;
    MotorControlSubscriber *motorControlSub;
    Motor motorLeft;
    Motor motorRight;
    bool pigpioNodeStarted;
    bool isStarted;

public:
    MotorControl(MotorControlSubscriber *motorControlSubscriber, MotorControlPublisher *motorControlPub, MotorControlClients *motorControlServiceClients);
    ~MotorControl() = default;
    void configureMotorGpios(void);
    void publishMessage(void);
    void edgeChangeCallback(const hal_pigpio::hal_pigpioEdgeChangeMsg &msg);
    bool isNotStarted(void);
    void starts(void);
    bool isPigpioNodeStarted(void);
    void pigpioHeartbeatCallback(const hal_pigpio::hal_pigpioHeartbeatMsg &msg);
};

#endif