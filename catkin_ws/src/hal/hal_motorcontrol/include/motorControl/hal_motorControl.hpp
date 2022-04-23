#ifndef HAL_MOTOR_CONTROL
#define HAL_MOTOR_CONTROL

#include "ros/ros.h"
#include "hal_motorControlVirtuals.hpp"
#include "commonDefinitions.hpp"
#include "hal_motor.hpp"

// Services and messages headers (generated)
#include "hal_pigpio/hal_pigpioSetInputMode.h"
#include "hal_pigpio/hal_pigpioSetOutputMode.h"
#include "hal_pigpio/hal_pigpioSetEncoderCallback.h"
#include "hal_pigpio/hal_pigpioSetPwmFrequency.h"
#include "hal_pigpio/hal_pigpioSetPwmDutycycle.h"
#include "hal_pigpio/hal_pigpioSetMotorDirection.h"
#include "hal_pigpio/hal_pigpioHeartbeatMsg.h"
#include "hal_motorcontrol/hal_motorControlMsg.h"

class MotorControl
{
private:
    MotorControlPublishers *motorControlPubs;
    MotorControlClients *motorControlClients;
    MotorControlSubscribers *motorControlSubs;
    Motor motorLeft;
    Motor motorRight;
    bool pigpioNodeStarted;
    bool isStarted;

public:
    MotorControl(MotorControlSubscribers *motorControlSubscribers, MotorControlPublishers *motorControlPublishers, MotorControlClients *motorControlServiceClients);
    ~MotorControl() = default;
    void configureMotor(void);
    void publishMessage(void);
    bool isNotStarted(void);
    void starts(void);
    bool isPigpioNodeStarted(void);
    void pigpioHeartbeatCallback(const hal_pigpio::hal_pigpioHeartbeatMsg &msg);

    void setPwmLeft(uint16_t dutycycle, bool direction);
    void setPwmRight(uint16_t dutycycle, bool direction);
};

#endif