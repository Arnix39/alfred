#ifndef HAL_MOTOR_CONTROL_VIRTUALS
#define HAL_MOTOR_CONTROL_VIRTUALS

#include "ros/ros.h"

// Services and messages headers (generated)
#include "hal_motorcontrol/hal_motorcontrolMsg.h"

class MotorControl;

class MotorControlPublishers
{
public:
    MotorControlPublishers() {}
    virtual ~MotorControlPublishers() {}
    virtual void publishMsg(hal_motorcontrol::hal_motorcontrolMsg message) = 0;
};

class MotorControlSubscribers
{
public:
    MotorControlSubscribers() {}
    virtual ~MotorControlSubscribers() {}
    virtual void subscribe(MotorControl *motorControl) = 0;
};

class MotorControlClients
{
public:
    MotorControlClients() {}
    virtual ~MotorControlClients() {}
    virtual ros::ServiceClient *getSetInputClientHandle() = 0;
    virtual ros::ServiceClient *getSetEncoderCallbackClientHandle() = 0;
    virtual ros::ServiceClient *getSetOutputClientHandle() = 0;
    virtual ros::ServiceClient *getSetPwmFrequencyClientHandle() = 0;
    virtual ros::ServiceClient *getSetPwmDutycycleClientHandle() = 0;
    virtual ros::ServiceClient *getSetMotorDirectionClientHandle() = 0;
};

#endif