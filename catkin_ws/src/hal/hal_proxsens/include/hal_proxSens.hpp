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

class ProxSens
{
private:
    ProxSensPublisher *proxSensPub;
    ros::ServiceClient gpioSetInputClient;
    ros::ServiceClient gpioSetCallbackClient;
    ros::ServiceClient gpioSetOutputClient;
    ros::ServiceClient gpioSendTriggerPulseClient;
    uint8_t edgeChangeType;
    uint32_t timestamp;
    uint32_t echoCallbackId;
    uint16_t distanceInCm;

public:
    ProxSens(ros::NodeHandle *node, ProxSensSubscriber *proxSensSubscriber, ProxSensPublisher *proxSensPub);
    void publishMessage(void);
    void configureGpios(void);
    void trigger(void);
    void edgeChangeCallback(const hal_pigpio::hal_pigpioEdgeChangeMsg &msg);
};

#endif