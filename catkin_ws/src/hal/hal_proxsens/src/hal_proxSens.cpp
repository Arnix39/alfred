#include "hal_proxSens.hpp"
#include "hal_proxSensInterfaces.hpp"

/* Publisher interface implementation */
ProxSensPublisherRos::ProxSensPublisherRos(ros::NodeHandle *node) : proxSensPubRos(node->advertise<hal_proxsens::hal_proxsensMsg>("proxSensorValue", 1000))
{
}

void ProxSensPublisherRos::publish(hal_proxsens::hal_proxsensMsg message)
{
    proxSensPubRos.publish(message);
}

/* Subscriber interface implementation */
ProxSensSubscriberRos::ProxSensSubscriberRos(ros::NodeHandle *node) : nodeHandle(node)
{
}

void ProxSensSubscriberRos::subscribe(ProxSens *proxSens)
{
    proxSensSubRos = nodeHandle->subscribe("gpioEdgeChange", 1000, &ProxSens::edgeChangeCallback, proxSens);
    proxSensPigpioHBSubRos = nodeHandle->subscribe("hal_pigpioHeartbeat", 1000, &ProxSens::pigpioHeartbeatCallback, proxSens);
}

/* Services interface implementation */
ProxSensClientsRos::ProxSensClientsRos(ros::NodeHandle *node) : gpioSetInputClientRos(node->serviceClient<hal_pigpio::hal_pigpioSetInputMode>("hal_pigpioSetInputMode")),
                                                                gpioSetOutputClientRos(node->serviceClient<hal_pigpio::hal_pigpioSetOutputMode>("hal_pigpioSetOutputMode")),
                                                                gpioSetCallbackClientRos(node->serviceClient<hal_pigpio::hal_pigpioSetCallback>("hal_pigpioSetCallback")),
                                                                gpioSendTriggerPulseClientRos(node->serviceClient<hal_pigpio::hal_pigpioSendTriggerPulse>("hal_pigpioSendTriggerPulse")),
                                                                gpioSetGpioHighClientRos(node->serviceClient<hal_pigpio::hal_pigpioSetGpioHigh>("hal_pigpioSetGpioHigh"))
{
}

ros::ServiceClient *ProxSensClientsRos::getSetInputClientHandle()
{
    return &gpioSetInputClientRos;
}

ros::ServiceClient *ProxSensClientsRos::getSetCallbackClientHandle()
{
    return &gpioSetCallbackClientRos;
}

ros::ServiceClient *ProxSensClientsRos::getSetOutputClientHandle()
{
    return &gpioSetOutputClientRos;
}

ros::ServiceClient *ProxSensClientsRos::getSendTriggerPulseClientHandle()
{
    return &gpioSendTriggerPulseClientRos;
}

ros::ServiceClient *ProxSensClientsRos::getSetGpioHighClientHandle()
{
    return &gpioSetGpioHighClientRos;
}

/* Proximity sensor implementation */
ProxSens::ProxSens(ProxSensSubscriber *proxSensSubscriber, ProxSensPublisher *proxSensPublisher, ProxSensClients *proxSensServiceClients) : edgeChangeType(NO_CHANGE),
                                                                                                                                            timestamp(0),
                                                                                                                                            echoCallbackId(0),
                                                                                                                                            distanceInCm(UINT16_MAX),
                                                                                                                                            pigpioNodeStarted(false),
                                                                                                                                            proxSensPub(proxSensPublisher),
                                                                                                                                            proxSensClients(proxSensServiceClients),
                                                                                                                                            proxSensSub(proxSensSubscriber)
{
    proxSensSub->subscribe(this);
}

void ProxSens::edgeChangeCallback(const hal_pigpio::hal_pigpioEdgeChangeMsg &msg)
{
    static uint8_t lastEdgeChangeType = NO_CHANGE;
    static uint32_t lastTimestamp = 0;

    uint32_t edgeLength = 0;

    if (msg.gpioId == PROXSENS_ECHO_GPIO)
    {
        edgeChangeType = msg.edgeChangeType;
        timestamp = msg.timeSinceBoot_us;

        if ((edgeChangeType == FALLING_EDGE) && (lastEdgeChangeType == RISING_EDGE))
        {
            if (timestamp < lastTimestamp)
            {
                edgeLength = UINT32_MAX - lastTimestamp + timestamp;
            }
            else
            {
                edgeLength = timestamp - lastTimestamp;
            }

            distanceInCm = (uint16_t)(edgeLength / 58.0);
        }
        else if (edgeChangeType == RISING_EDGE)
        {
            lastEdgeChangeType = edgeChangeType;
            lastTimestamp = timestamp;
        }
    }
}

void ProxSens::pigpioHeartbeatCallback(const hal_pigpio::hal_pigpioHeartbeatMsg &msg)
{
    pigpioNodeStarted = msg.isAlive;
}

bool ProxSens::isPigpioNodeStarted(void)
{
    return pigpioNodeStarted;
}

void ProxSens::publishMessage(void)
{
    hal_proxsens::hal_proxsensMsg message;

    message.distanceInCm = distanceInCm;
    proxSensPub->publish(message);
}

void ProxSens::configureGpios(void)
{
    hal_pigpio::hal_pigpioSetInputMode setInputModeSrv;
    hal_pigpio::hal_pigpioSetCallback setCallbackSrv;
    hal_pigpio::hal_pigpioSetOutputMode setOutputModeSrv;

    setInputModeSrv.request.gpioId = PROXSENS_ECHO_GPIO;
    proxSensClients->getSetInputClientHandle()->call(setInputModeSrv);

    setCallbackSrv.request.gpioId = PROXSENS_ECHO_GPIO;
    setCallbackSrv.request.edgeChangeType = AS_EITHER_EDGE;
    proxSensClients->getSetCallbackClientHandle()->call(setCallbackSrv);
    if (setCallbackSrv.response.hasSucceeded)
    {
        echoCallbackId = setCallbackSrv.response.callbackId;
    }

    setOutputModeSrv.request.gpioId = PROXSENS_TRIGGER_GPIO;
    proxSensClients->getSetOutputClientHandle()->call(setOutputModeSrv);

    setOutputModeSrv.request.gpioId = PROXSENS_LEVEL_SHIFTER_OE_GPIO;
    proxSensClients->getSetOutputClientHandle()->call(setOutputModeSrv);
}

void ProxSens::trigger(void)
{
    hal_pigpio::hal_pigpioSendTriggerPulse sendTriggerPulseSrv;

    sendTriggerPulseSrv.request.gpioId = PROXSENS_TRIGGER_GPIO;
    sendTriggerPulseSrv.request.pulseLengthInUs = PROXSENS_TRIGGER_LENGTH_US;

    proxSensClients->getSendTriggerPulseClientHandle()->call(sendTriggerPulseSrv);
}

void ProxSens::enableOutputLevelShifter(void)
{
    hal_pigpio::hal_pigpioSetGpioHigh setGpioHighSrv;

    setGpioHighSrv.request.gpioId = PROXSENS_LEVEL_SHIFTER_OE_GPIO;

    proxSensClients->getSetGpioHighClientHandle()->call(setGpioHighSrv);
}

void ProxSens::publishAndGetDistance(const ros::TimerEvent &timerEvent)
{
    publishMessage();
    trigger();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hal_proxsens");
    ros::NodeHandle node;
    ros::Timer proxSensTimer;

    ProxSensPublisherRos proxSensPublisherRos(&node);
    ProxSensSubscriberRos proxSensSubscriberRos(&node);
    ProxSensClientsRos proxSensServiceClientsRos(&node);

    ProxSens proxSens(&proxSensSubscriberRos, &proxSensPublisherRos, &proxSensServiceClientsRos);

    ROS_INFO("proxSens node waiting for pigpio node to start...");
    while (ros::ok())
    {
        if (proxSens.isPigpioNodeStarted())
        {
            ROS_INFO("proxSens node initialising...");
            proxSens.configureGpios();
            proxSens.enableOutputLevelShifter();
            proxSensTimer = node.createTimer(ros::Duration(0.1), &ProxSens::publishAndGetDistance, &proxSens);
            ROS_INFO("proxSens node initialised.");
        }

        ros::spinOnce();
    }

    return 0;
}