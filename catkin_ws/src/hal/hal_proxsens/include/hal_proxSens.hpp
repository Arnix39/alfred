#ifndef HAL_PROXSENS
#define HAL_PROXSENS

#include "stdint.h"

#include "ros/ros.h"

// Services and messages headers (generated)
#include "hal_pigpio/hal_pigpioSetInputMode.h"
#include "hal_pigpio/hal_pigpioSetOutputMode.h"
#include "hal_pigpio/hal_pigpioSetGpioHigh.h"
#include "hal_pigpio/hal_pigpioSetGpioLow.h"
#include "hal_pigpio/hal_pigpioSendTriggerPulse.h"
#include "hal_pigpio/hal_pigpioSetCallback.h"
#include "hal_pigpio/hal_pigpioEdgeChangeMsg.h"
#include "hal_proxsens/hal_proxsensMsg.h"

#define PROXSENS_TRIGGER_GPIO 5
#define PROXSENS_ECHO_GPIO 6
#define PROXSENS_TRIGGER_LENGTH_US 20

#define AS_RISING_EDGE 0
#define AS_FALLING_EDGE 1
#define AS_EITHER_EDGE 2

#define FALLING_EDGE 0
#define RISING_EDGE 1
#define NO_CHANGE 2

class ProxSens;

class ProxSensPublisher
{
public:
    ProxSensPublisher() {}
    virtual ~ProxSensPublisher() {}
    virtual void publish(hal_proxsens::hal_proxsensMsg message) = 0;
};

class ProxSensPublisherRos : public ProxSensPublisher
{
private:
    ros::Publisher proxSensPubRos;

public:
    ProxSensPublisherRos(ros::NodeHandle *node);
    ~ProxSensPublisherRos() = default;
    void publish(hal_proxsens::hal_proxsensMsg message) override;
};

class ProxSensSubscriber
{
public:
    ProxSensSubscriber() {}
    virtual ~ProxSensSubscriber() {}
    virtual void subscribe(ProxSens *proxSens) = 0;
};

class ProxSensSubscriberRos : public ProxSensSubscriber
{
private:
    ros::Subscriber proxSensSubRos;
    ros::NodeHandle *nodeHandle;

public:
    ProxSensSubscriberRos(ros::NodeHandle *node);
    ~ProxSensSubscriberRos() = default;
    void subscribe(ProxSens *proxSens) override;
};

class ProxSensClients
{
public:
    ProxSensClients() {}
    virtual ~ProxSensClients() {}
    virtual ros::ServiceClient getSetInputClientHandle() = 0;
    virtual ros::ServiceClient getSetCallbackClientHandle() = 0;
    virtual ros::ServiceClient getSetOutputClientHandle() = 0;
    virtual ros::ServiceClient getSendTriggerPulseClientHandle() = 0;
};

class ProxSensClientsRos : public ProxSensClients
{
private:
    ros::ServiceClient gpioSetInputClientRos;
    ros::ServiceClient gpioSetCallbackClientRos;
    ros::ServiceClient gpioSetOutputClientRos;
    ros::ServiceClient gpioSendTriggerPulseClientRos;

public:
    ProxSensClientsRos(ros::NodeHandle *node);
    ~ProxSensClientsRos() = default;
    ros::ServiceClient getSetInputClientHandle() override;
    ros::ServiceClient getSetCallbackClientHandle() override;
    ros::ServiceClient getSetOutputClientHandle() override;
    ros::ServiceClient getSendTriggerPulseClientHandle() override;
};

class ProxSens
{
private:
    ProxSensPublisher *proxSensPub;
    ProxSensClients *proxSensClients;
    uint8_t edgeChangeType;
    uint32_t timestamp;
    uint32_t echoCallbackId;
    uint16_t distanceInCm;

public:
    ProxSens(ProxSensSubscriber *proxSensSubscriber, ProxSensPublisher *proxSensPub, ProxSensClients *proxSensServiceClients);
    void publishMessage(void);
    void configureGpios(void);
    void trigger(void);
    void edgeChangeCallback(const hal_pigpio::hal_pigpioEdgeChangeMsg &msg);
};

#endif