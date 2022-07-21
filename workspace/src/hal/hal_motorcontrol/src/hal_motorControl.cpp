#include "hal_motorControl.hpp"
#include "hal_motorControlInterfaces.hpp"

/* Publishers interface implementation */
MotorControlPublishersRos::MotorControlPublishersRos(ros::NodeHandle *node) : motorControlPubRos(node->advertise<hal_motorcontrol::hal_motorControlMsg>("motorsEncoderCountValue", 1000))
{
}

void MotorControlPublishersRos::publishEncoderCounts(hal_motorcontrol::hal_motorControlMsg message)
{
    motorControlPubRos.publish(message);
}

/* Subscriber interface implementation */
MotorControlSubscribersRos::MotorControlSubscribersRos(ros::NodeHandle *node) : nodeHandle(node)
{
}

void MotorControlSubscribersRos::subscribe(MotorControl *motorControl)
{
    motorControlPigpioHBSubRos = nodeHandle->subscribe("hal_pigpioHeartbeat", 1000, &MotorControl::pigpioHeartbeatCallback, motorControl);
    motorControlPigpioECSubRos = nodeHandle->subscribe("hal_pigpioEncoderCount", 1000, &MotorControl::pigpioEncoderCountCallback, motorControl);
}

/* Services interface implementation */
MotorControlClientsRos::MotorControlClientsRos(ros::NodeHandle *node) : gpioSetInputClientRos(node->serviceClient<hal_pigpio::hal_pigpioSetInputMode>("hal_pigpioSetInputMode")),
                                                                        gpioSetOutputClientRos(node->serviceClient<hal_pigpio::hal_pigpioSetOutputMode>("hal_pigpioSetOutputMode")),
                                                                        gpioSetEncoderCallbackClientRos(node->serviceClient<hal_pigpio::hal_pigpioSetEncoderCallback>("hal_pigpioSetEncoderCallback")),
                                                                        gpioSetPwmFrequencyClientRos(node->serviceClient<hal_pigpio::hal_pigpioSetPwmFrequency>("hal_pigpioSetPwmFrequency")),
                                                                        gpioSetPwmDutycycleClientRos(node->serviceClient<hal_pigpio::hal_pigpioSetPwmDutycycle>("hal_pigpioSetPwmDutycycle")),
                                                                        gpioSetMotorDirectionClientRos(node->serviceClient<hal_pigpio::hal_pigpioSetMotorDirection>("hal_pigpioSetMotorDirection"))
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

ros::ServiceClient *MotorControlClientsRos::getSetMotorDirectionClientHandle()
{
    return &gpioSetMotorDirectionClientRos;
}

/* Motor control implementation */
MotorControl::MotorControl(MotorControlSubscribers *motorControlSubscribers,
                           MotorControlPublishers *motorControlPublishers, 
                           MotorControlClients *motorControlServiceClients) : motorControlPubs(motorControlPublishers),
                                                                              motorControlClients(motorControlServiceClients),
                                                                              motorControlSubs(motorControlSubscribers),
                                                                              motorLeft(MOTOR_LEFT_PWM_A_GPIO, MOTOR_LEFT_PWM_B_GPIO,
                                                                                        MOTOR_LEFT_ENCODER_CH_A_GPIO, MOTOR_LEFT_ENCODER_CH_B_GPIO,
                                                                                        MOTOR_LEFT),
                                                                              motorRight(MOTOR_RIGHT_PWM_A_GPIO, MOTOR_RIGHT_PWM_B_GPIO,
                                                                                         MOTOR_RIGHT_ENCODER_CH_A_GPIO, MOTOR_RIGHT_ENCODER_CH_B_GPIO,
                                                                                         MOTOR_RIGHT)
{
    motorControlSubs->subscribe(this);
}

void MotorControl::configureMotor(void)
{
    motorLeft.configureGpios(motorControlClients->getSetOutputClientHandle(), motorControlClients->getSetInputClientHandle(), 
                             motorControlClients->getSetEncoderCallbackClientHandle(), motorControlClients->getSetPwmFrequencyClientHandle());
    motorLeft.configureSetPwmDutycycleClientHandle(motorControlClients->getSetPwmDutycycleClientHandle());
    motorLeft.configureSetMotorDirectionClientHandle(motorControlClients->getSetMotorDirectionClientHandle());

    motorRight.configureGpios(motorControlClients->getSetOutputClientHandle(), motorControlClients->getSetInputClientHandle(), 
                              motorControlClients->getSetEncoderCallbackClientHandle(), motorControlClients->getSetPwmFrequencyClientHandle());
    motorRight.configureSetPwmDutycycleClientHandle(motorControlClients->getSetPwmDutycycleClientHandle());
    motorRight.configureSetMotorDirectionClientHandle(motorControlClients->getSetMotorDirectionClientHandle());
}

void MotorControl::pigpioEncoderCountCallback(const hal_pigpio::hal_pigpioEncoderCountMsg &msg)
{
    if (msg.motorId == MOTOR_LEFT)
    {
        motorLeft.setEncoderCount(msg.encoderCount);
    }
    else if (msg.motorId == MOTOR_RIGHT)
    {
        motorRight.setEncoderCount(msg.encoderCount);
    }
    else
    {
        RCLCPP_ERROR(get_logger(), "Encoder count message received for unknown motor!");
    }
    
}

void MotorControl::publishMessage(const ros::TimerEvent &timerEvent)
{
    hal_motorcontrol::hal_motorControlMsg message;

    message.motorLeftEncoderCount = motorLeft.getEncoderCount();
    message.motorRightEncoderCount = motorRight.getEncoderCount();
    motorControlPubs->publishEncoderCounts(message);
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
    motorLeft.setPwmDutyCycleAndDirection(dutycycle, direction);
}

void MotorControl::setPwmRight(uint16_t dutycycle, bool direction)
{
    motorRight.setPwmDutyCycleAndDirection(dutycycle, direction);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hal_motorcontrol");
    ros::NodeHandle node;

    MotorControlSubscribersRos motorControlSubscribersRos(&node);
    MotorControlPublishersRos motorControlPublishersRos(&node);
    MotorControlClientsRos motorControlServiceClientsRos(&node);

    MotorControl motorControl(&motorControlSubscribersRos, &motorControlPublishersRos, &motorControlServiceClientsRos);

    ros::Timer motorControlECTimer(node.createTimer(ros::Duration(0.05), &MotorControl::publishMessage, &motorControl));

    ros::Rate rate(1);

    RCLCPP_INFO(get_logger(), "motorControl node waiting for pigpio node to start...");
    while (ros::ok())
    {
        if (motorControl.isNotStarted() && motorControl.isPigpioNodeStarted())
        {
            RCLCPP_INFO(get_logger(), "motorControl node initialising...");
            motorControl.configureMotor();
            motorControl.starts();
            RCLCPP_INFO(get_logger(), "motorControl node initialised.");
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

            RCLCPP_INFO(get_logger(), "Dutycycle: %d.", dutycycle);

            motorControl.setPwmLeft(dutycycle, direction);
            motorControl.setPwmRight(dutycycle, direction);

            dutycycle = dutycycle + 10;
        }

        rate.sleep();
        ros::spinOnce();
    }

    return 0;
}