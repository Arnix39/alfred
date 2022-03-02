#ifndef HAL_MOTORCONTROLVIRTUALS
#define HAL_MOTORCONTROLVIRTUALS

#include "ros/ros.h"

// Services and messages headers (generated)
#include "hal_motorcontrol/hal_motorcontrolMsg.h"

class MotorControl;

class MotorControlPublisher
{
public:
    MotorControlPublisher() {}
    virtual ~MotorControlPublisher() {}
    virtual void publish(hal_motorcontrol::hal_motorcontrolMsg message) = 0;
};

class MotorControlSubscriber
{
public:
    MotorControlSubscriber() {}
    virtual ~MotorControlSubscriber() {}
    virtual void subscribe(MotorControl *motorControl) = 0;
};

class MotorControlClients
{
public:
    MotorControlClients() {}
    virtual ~MotorControlClients() {}
    virtual ros::ServiceClient *getSetInputClientHandle() = 0;
    virtual ros::ServiceClient *getSetCallbackClientHandle() = 0;
    virtual ros::ServiceClient *getSetOutputClientHandle() = 0;
    virtual ros::ServiceClient *getSetPwmFrequencyClientHandle() = 0;
    virtual ros::ServiceClient *getSetPwmDutycycleClientHandle() = 0;
};

#endif