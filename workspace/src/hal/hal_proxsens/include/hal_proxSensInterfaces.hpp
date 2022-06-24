#ifndef HAL_PROXSENS_INTERFACES
#define HAL_PROXSENS_INTERFACES

#include "hal_proxSensVirtuals.hpp"

class ProxSensPublisherRos : public ProxSensPublisher
{
private:
    rclcpp::Publisher proxSensPubRos;

public:
    ProxSensPublisherRos(rclcpp::NodeHandle *node);
    ~ProxSensPublisherRos() = default;
    void publish(hal_proxsens::hal_proxsensMsg message) override;
};

class ProxSensSubscriberRos : public ProxSensSubscriber
{
private:
    rclcpp::Subscriber proxSensSubRos;
    rclcpp::Subscriber proxSensPigpioHBSubRos;
    rclcpp::NodeHandle *nodeHandle;

public:
    ProxSensSubscriberRos(rclcpp::NodeHandle *node);
    ~ProxSensSubscriberRos() = default;
    void subscribe(ProxSens *proxSens) override;
};

class ProxSensClientsRos : public ProxSensClients
{
private:
    rclcpp::ServiceClient gpioSetInputClientRos;
    rclcpp::ServiceClient gpioSetCallbackClientRos;
    rclcpp::ServiceClient gpioSetOutputClientRos;
    rclcpp::ServiceClient gpioSendTriggerPulseClientRos;
    rclcpp::ServiceClient gpioSetGpioHighClientRos;

public:
    ProxSensClientsRos(rclcpp::NodeHandle *node);
    ~ProxSensClientsRos() = default;
    rclcpp::ServiceClient *getSetInputClientHandle() override;
    rclcpp::ServiceClient *getSetCallbackClientHandle() override;
    rclcpp::ServiceClient *getSetOutputClientHandle() override;
    rclcpp::ServiceClient *getSendTriggerPulseClientHandle() override;
    rclcpp::ServiceClient *getSetGpioHighClientHandle() override;
};

#endif