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
    motorControlPigpioHBSubRos = nodeHandle->subscribe("hal_pigpioHeartbeat", 1000, &MotorControl::pigpioHeartbeatCallback, motorControl);
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
MotorControl::MotorControl(MotorControlSubscriber *motorControlSubscriber,
                           MotorControlPublisher *motorControlPublisher, 
                           MotorControlClients *motorControlServiceClients) : motorControlPub(motorControlPublisher),
                                                                              motorControlClients(motorControlServiceClients),
                                                                              motorControlSub(motorControlSubscriber),
                                                                              motorLeft(MOTOR_LEFT_PWM_A_GPIO, MOTOR_LEFT_PWM_B_GPIO,
                                                                                        MOTOR_LEFT_ENCODER_CH_A_GPIO, MOTOR_LEFT_ENCODER_CH_B_GPIO),
                                                                              motorRight(MOTOR_RIGHT_PWM_A_GPIO, MOTOR_RIGHT_PWM_B_GPIO,
                                                                                         MOTOR_RIGHT_ENCODER_CH_A_GPIO, MOTOR_RIGHT_ENCODER_CH_B_GPIO)
{
    motorControlSub->subscribe(this);
}

void MotorControl::configureMotorGpios(void)
{
    motorLeft.configureGpios(motorControlClients->getSetOutputClientHandle(), motorControlClients->getSetInputClientHandle(), motorControlClients->getSetCallbackClientHandle());
    motorRight.configureGpios(motorControlClients->getSetOutputClientHandle(), motorControlClients->getSetInputClientHandle(), motorControlClients->getSetCallbackClientHandle());
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

void MotorControl::starts(void)
{
    isStarted = true;
}

bool MotorControl::isNotStarted(void)
{
    return !isStarted;
}

void MotorControl::pigpioHeartbeatCallback(const hal_pigpio::hal_pigpioHeartbeatMsg &msg)
{
    pigpioNodeStarted = msg.isAlive;
}

bool MotorControl::isPigpioNodeStarted(void)
{
    return pigpioNodeStarted;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hal_motorcontrol");
    ros::NodeHandle node;

    MotorControlPublisherRos motorControlPublisherRos(&node);
    MotorControlSubscriberRos motorControlSubscriberRos(&node);
    MotorControlClientsRos motorControlServiceClientsRos(&node);

    MotorControl motorControl(&motorControlSubscriberRos, &motorControlPublisherRos, &motorControlServiceClientsRos);

    ROS_INFO("motorControl node waiting for pigpio node to start...");
    while (ros::ok())
    {
        if (motorControl.isNotStarted() && motorControl.isPigpioNodeStarted())
        {
            ROS_INFO("motorControl node initialising...");
            motorControl.configureMotorGpios();
            motorControl.starts();
            ROS_INFO("motorControl node initialised.");
        }

        ros::spinOnce();
    }

    return 0;
}