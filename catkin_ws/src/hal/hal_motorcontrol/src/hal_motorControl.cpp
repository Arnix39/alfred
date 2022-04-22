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
    motorControlPigpioHBSubRos = nodeHandle->subscribe("hal_pigpioHeartbeat", 1000, &MotorControl::pigpioHeartbeatCallback, motorControl);
}

/* Services interface implementation */
MotorControlClientsRos::MotorControlClientsRos(ros::NodeHandle *node) : gpioSetInputClientRos(node->serviceClient<hal_pigpio::hal_pigpioSetInputMode>("hal_pigpioSetInputMode")),
                                                                        gpioSetOutputClientRos(node->serviceClient<hal_pigpio::hal_pigpioSetOutputMode>("hal_pigpioSetOutputMode")),
                                                                        gpioSetEncoderCallbackClientRos(node->serviceClient<hal_pigpio::hal_pigpioSetEncoderCallback>("hal_pigpioSetEncoderCallback")),
                                                                        gpioSetPwmFrequencyClientRos(node->serviceClient<hal_pigpio::hal_pigpioSetPwmFrequency>("hal_pigpioSetPwmFrequency")),
                                                                        gpioSetPwmDutycycleClientRos(node->serviceClient<hal_pigpio::hal_pigpioSetPwmDutycycle>("hal_pigpioSetPwmDutycycle"))
{
}

ros::ServiceClient *MotorControlClientsRos::getSetInputClientHandle()
{
    return &gpioSetInputClientRos;
}

ros::ServiceClient *MotorControlClientsRos::getSetOutputClientHandle()
{
    return &gpioSetOutputClientRos;
}

ros::ServiceClient *MotorControlClientsRos::getSetEncoderCallbackClientHandle()
{
    return &gpioSetEncoderCallbackClientRos;
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
                                                                                        MOTOR_LEFT_ENCODER_CH_A_GPIO, MOTOR_LEFT_ENCODER_CH_B_GPIO,
                                                                                        MOTOR_LEFT),
                                                                              motorRight(MOTOR_RIGHT_PWM_A_GPIO, MOTOR_RIGHT_PWM_B_GPIO,
                                                                                         MOTOR_RIGHT_ENCODER_CH_A_GPIO, MOTOR_RIGHT_ENCODER_CH_B_GPIO,
                                                                                         MOTOR_RIGHT)
{
    motorControlSub->subscribe(this);
}

void MotorControl::configureMotor(void)
{
    motorLeft.configureGpios(motorControlClients->getSetOutputClientHandle(), motorControlClients->getSetInputClientHandle(), 
                             motorControlClients->getSetEncoderCallbackClientHandle(), motorControlClients->getSetPwmFrequencyClientHandle(),
                             motorLeft.getId());
    motorLeft.configureSetPwmDutycycleClientHandle(motorControlClients->getSetPwmDutycycleClientHandle());

    motorRight.configureGpios(motorControlClients->getSetOutputClientHandle(), motorControlClients->getSetInputClientHandle(), 
                              motorControlClients->getSetEncoderCallbackClientHandle(), motorControlClients->getSetPwmFrequencyClientHandle(),
                              motorRight.getId());
    motorRight.configureSetPwmDutycycleClientHandle(motorControlClients->getSetPwmDutycycleClientHandle());
}

void MotorControl::publishMessage(void)
{
    hal_motorcontrol::hal_motorcontrolMsg message;

    message.motorLeftEncoderCount = motorLeft.getEncoderCount();
    message.motorRightEncoderCount = motorRight.getEncoderCount();
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

void MotorControl::setPwmLeft(uint16_t dutycycle, bool direction)
{
    motorLeft.setPwmDutyCycle(dutycycle, direction);
}

void MotorControl::setPwmRight(uint16_t dutycycle, bool direction)
{
    motorRight.setPwmDutyCycle(dutycycle, direction);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hal_motorcontrol");
    ros::NodeHandle node;

    MotorControlPublisherRos motorControlPublisherRos(&node);
    MotorControlSubscriberRos motorControlSubscriberRos(&node);
    MotorControlClientsRos motorControlServiceClientsRos(&node);

    MotorControl motorControl(&motorControlSubscriberRos, &motorControlPublisherRos, &motorControlServiceClientsRos);

    ros::Rate rate(1);

    ROS_INFO("motorControl node waiting for pigpio node to start...");
    while (ros::ok())
    {
        if (motorControl.isNotStarted() && motorControl.isPigpioNodeStarted())
        {
            ROS_INFO("motorControl node initialising...");
            motorControl.configureMotor();
            motorControl.starts();
            ROS_INFO("motorControl node initialised.");
        }
        else if(!motorControl.isNotStarted())
        {
            static uint16_t dutycycle = 0;
            static bool direction = true;

            if(dutycycle > 255)
            {
                direction = !direction;
                dutycycle = 0;
            }

            ROS_INFO("Dutycycle: %d.", dutycycle);

            motorControl.setPwmLeft(dutycycle, direction);
            motorControl.setPwmRight(dutycycle, direction);

            dutycycle = dutycycle + 10;
        }

        rate.sleep();
        ros::spinOnce();
    }

    return 0;
}