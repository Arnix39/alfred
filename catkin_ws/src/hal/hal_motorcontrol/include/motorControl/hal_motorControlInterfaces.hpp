#ifndef HAL_MOTOR_CONTROL_INTERFACES
#define HAL_MOTOR_CONTROL_INTERFACES

#include "hal_motorControlVirtuals.hpp"

class MotorControlPublishersRos : public MotorControlPublishers
{
private:
    ros::Publisher motorControlPubRos;

public:
    MotorControlPublishersRos(ros::NodeHandle *node);
    ~MotorControlPublishersRos() = default;
    void publishMsg(hal_motorcontrol::hal_motorcontrolMsg message) override;
};

class MotorControlSubscribersRos : public MotorControlSubscribers
{
private:
    ros::Subscriber motorControlPigpioHBSubRos;
    ros::NodeHandle *nodeHandle;

public:
    MotorControlSubscribersRos(ros::NodeHandle *node);
    ~MotorControlSubscribersRos() = default;
    void subscribe(MotorControl *motorControl) override;
};

class MotorControlClientsRos : public MotorControlClients
{
private:
    ros::ServiceClient gpioSetInputClientRos;
    ros::ServiceClient gpioSetEncoderCallbackClientRos;
    ros::ServiceClient gpioSetOutputClientRos;
    ros::ServiceClient gpioSetPwmFrequencyClientRos;
    ros::ServiceClient gpioSetPwmDutycycleClientRos;
    ros::ServiceClient gpioSetMotorDirectionClientRos;

public:
    MotorControlClientsRos(ros::NodeHandle *node);
    ~MotorControlClientsRos() = default;
    ros::ServiceClient *getSetInputClientHandle() override;
    ros::ServiceClient *getSetEncoderCallbackClientHandle() override;
    ros::ServiceClient *getSetOutputClientHandle() override;
    ros::ServiceClient *getSetPwmFrequencyClientHandle() override;
    ros::ServiceClient *getSetPwmDutycycleClientHandle() override;
    ros::ServiceClient *getSetMotorDirectionClientHandle() override;
};

#endif