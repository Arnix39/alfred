#ifndef HAL_MOTORCONTROL
#define HAL_MOTORCONTROL

#include "ros/ros.h"
#include "hal_motorControlVirtuals.hpp"

// Services and messages headers (generated)
#include "hal_pigpio/hal_pigpioSetInputMode.h"
#include "hal_pigpio/hal_pigpioSetOutputMode.h"
#include "hal_pigpio/hal_pigpioSetCallback.h"
#include "hal_pigpio/hal_pigpioSetPwmFrequency.h"
#include "hal_pigpio/hal_pigpioSetPwmDutycycle.h"
#include "hal_pigpio/hal_pigpioEdgeChangeMsg.h"
#include "hal_motorcontrol/hal_motorcontrolMsg.h"

#define MOTOR_LEFT_PWM_A_GPIO 9
#define MOTOR_LEFT_PWM_B_GPIO 25
#define MOTOR_LEFT_ENCODER_CH_A_GPIO 17
#define MOTOR_LEFT_ENCODER_CH_B_GPIO 18

#define MOTOR_RIGHT_PWM_A_GPIO 11
#define MOTOR_RIGHT_PWM_B_GPIO 8
#define MOTOR_RIGHT_ENCODER_CH_A_GPIO 22
#define MOTOR_RIGHT_ENCODER_CH_B_GPIO 23

#define AS_RISING_EDGE 0
#define AS_FALLING_EDGE 1
#define AS_EITHER_EDGE 2

#define FALLING_EDGE 0
#define RISING_EDGE 1
#define NO_CHANGE 2

class MotorControl
{
private:
    MotorControlPublisher *motorControlPub;
    MotorControlClients *motorControlClients;
    MotorControlSubscriber *motorControlSub;
    uint8_t edgeChangeType;

public:
    MotorControl(MotorControlSubscriber *motorControlSubscriber, MotorControlPublisher *motorControlPub, MotorControlClients *motorControlServiceClients);
    ~MotorControl() = default;
    void publishMessage(void);
    void edgeChangeCallback(const hal_pigpio::hal_pigpioEdgeChangeMsg &msg);
};

#endif