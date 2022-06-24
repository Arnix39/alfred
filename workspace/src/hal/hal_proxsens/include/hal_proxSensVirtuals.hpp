#ifndef HAL_PROXSENS_VIRTUALS
#define HAL_PROXSENS_VIRTUALS

#include "rclcpp/rclcpp.hpp"

// Services and messages headers (generated)
#include "hal_proxsens/msg/hal_proxsens.hpp"

class ProxSens;

class ProxSensPublisher
{
public:
    ProxSensPublisher() {}
    virtual ~ProxSensPublisher() {}
    virtual void publish(hal_proxsens::msg::HalProxsens message) = 0;
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
    virtual rclcpp::ServiceClient *getSetInputClientHandle() = 0;
    virtual rclcpp::ServiceClient *getSetCallbackClientHandle() = 0;
    virtual rclcpp::ServiceClient *getSetOutputClientHandle() = 0;
    virtual rclcpp::ServiceClient *getSendTriggerPulseClientHandle() = 0;
    virtual rclcpp::ServiceClient *getSetGpioHighClientHandle() = 0;
};

#endif