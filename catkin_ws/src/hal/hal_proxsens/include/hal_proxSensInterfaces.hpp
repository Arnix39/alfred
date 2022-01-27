#ifndef HAL_PROXSENSINTERFACES
#define HAL_PROXSENSINTERFACES

#include "hal_proxSensVirtuals.hpp"

class ProxSensPublisherRos : public ProxSensPublisher
{
private:
    ros::Publisher proxSensPubRos;

public:
    ProxSensPublisherRos(ros::NodeHandle *node);
    ~ProxSensPublisherRos() = default;
    void publish(hal_proxsens::hal_proxsensMsg message) override;
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

#endif