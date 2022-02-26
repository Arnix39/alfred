#include "hal_motorControl.hpp"
#include "hal_motorControlInterfaces.hpp"

/* Publisher interface implementation */
MotorControlPublisherRos::MotorControlPublisherRos(ros::NodeHandle *node) : motorControlPubRos(node->advertise<hal_motorcontrol::hal_motorcontrolMsg>("motorsEncoderCountValue", 1000))
{
}

void MotorControlPublisherRos::publish(hal_motorcontrol::hal_motorcontrolMsg message)
{
    motorControlPubRos.publish(message);
}

/* Subscriber interface implementation */
MotorControlSubscriberRos::MotorControlSubscriberRos(ros::NodeHandle *node) : nodeHandle(node)
{
}

void MotorControlSubscriberRos::subscribe(MotorControl *motorControl)
{
    // TODO: once EC messages from motor classes is implemented, change these calls
    motorLeftSubRos = nodeHandle->subscribe("gpioEdgeChange", 1000, &MotorControl::edgeChangeCallback, motorControl);
    motorRightSubRos = nodeHandle->subscribe("gpioEdgeChange", 1000, &MotorControl::edgeChangeCallback, motorControl);
}

/* Services interface implementation */
MotorControlClientsRos::MotorControlClientsRos(ros::NodeHandle *node) : gpioSetInputClientRos(node->serviceClient<hal_pigpio::hal_pigpioSetInputMode>("hal_pigpioSetInputMode")),
                                                                        gpioSetOutputClientRos(node->serviceClient<hal_pigpio::hal_pigpioSetOutputMode>("hal_pigpioSetOutputMode")),
                                                                        gpioSetCallbackClientRos(node->serviceClient<hal_pigpio::hal_pigpioSetCallback>("hal_pigpioSetCallback")),
                                                                        gpioSetPwmFrequencyClientRos(node->serviceClient<hal_pigpio::hal_pigpioSetPwmFrequency>("hal_pigpioSetPwmFrequency")),
                                                                        gpioSetPwmDutycycleClientRos(node->serviceClient<hal_pigpio::hal_pigpioSetPwmDutycycle>("hal_pigpioSetPwmDutycycle"))
{
}

ros::ServiceClient *MotorControlClientsRos::getSetInputClientHandle()
{
    return &gpioSetInputClientRos;
}

ros::ServiceClient *MotorControlClientsRos::getSetCallbackClientHandle()
{
    return &gpioSetCallbackClientRos;
}

ros::ServiceClient *MotorControlClientsRos::getSetOutputClientHandle()
{
    return &gpioSetOutputClientRos;
}

ros::ServiceClient *MotorControlClientsRos::getSetPwmFrequencyClientHandle()
{
    return &gpioSetPwmFrequencyClientRos;
}

ros::ServiceClient *MotorControlClientsRos::getSetPwmDutycycleClientHandle()
{
    return &gpioSetPwmDutycycleClientRos;
}

/* Motor control implementation */
MotorControl::MotorControl(MotorControlSubscriber *motorControlSubscriber, MotorControlPublisher *motorControlPublisher, MotorControlClients *motorControlServiceClients) : rightEncoderCount(0),
                                                                                                                                                                            leftEncoderCount(0),
                                                                                                                                                                            motorControlPub(motorControlPublisher),
                                                                                                                                                                            motorControlClients(motorControlServiceClients),
                                                                                                                                                                            motorControlSub(motorControlSubscriber)
{
    motorControlSub->subscribe(this);
}

void MotorControl::edgeChangeCallback(const hal_pigpio::hal_pigpioEdgeChangeMsg &msg)
{
}

void MotorControl::publishMessage(void)
{
    hal_motorcontrol::hal_motorcontrolMsg message;

    message.motorLeftEncoderCount = 22;
    message.motorRightEncoderCount = 22;
    motorControlPub->publish(message);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hal_motorcontrol");
    ros::NodeHandle node;

    MotorControlPublisherRos motorControlPublisherRos(&node);
    MotorControlSubscriberRos motorControlSubscriberRos(&node);
    MotorControlClientsRos motorControlServiceClientsRos(&node);

    MotorControl motorControl(&motorControlSubscriberRos, &motorControlPublisherRos, &motorControlServiceClientsRos);

    ros::Rate loop_rate(100);

    while (ros::ok())
    {
        motorControl.publishMessage();

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}