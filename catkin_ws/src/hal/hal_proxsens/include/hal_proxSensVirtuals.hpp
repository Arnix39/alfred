#ifndef HAL_PROXSENSVIRTUALS
#define HAL_PROXSENSVIRTUALS

#include "ros/ros.h"

// Services and messages headers (generated)
#include "hal_proxsens/hal_proxsensMsg.h"

class ProxSens;

class ProxSensPublisher
{
public:
    ProxSensPublisher() {}
    virtual ~ProxSensPublisher() {}
    virtual void publish(hal_proxsens::hal_proxsensMsg message) = 0;
};

class ProxSensSubscriber
{
public:
    ProxSensSubscriber() {}
    virtual ~ProxSensSubscriber() {}
    virtual void subscribe(ProxSens *proxSens) = 0;
};

class ProxSensClients
{
public:
    ProxSensClients() {}
    virtual ~ProxSensClients() {}
    virtual ros::ServiceClient *getSetInputClientHandle() = 0;
    virtual ros::ServiceClient *getSetCallbackClientHandle() = 0;
    virtual ros::ServiceClient *getSetOutputClientHandle() = 0;
    virtual ros::ServiceClient *getSendTriggerPulseClientHandle() = 0;
};

#endif