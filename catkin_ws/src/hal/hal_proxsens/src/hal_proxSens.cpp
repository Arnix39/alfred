#include "hal_proxSens.hpp"
#include "hal_proxSensInterfaces.hpp"

/* Publisher interface implementation */
ProxSensPublisherRos::ProxSensPublisherRos(ros::NodeHandle *node)
{
    proxSensPubRos = node->advertise<hal_proxsens::hal_proxsensMsg>("proxSensorValue", 1000);
}

void ProxSensPublisherRos::publish(hal_proxsens::hal_proxsensMsg message)
{
    proxSensPubRos.publish(message);
}

/* Subscriber interface implementation */
ProxSensSubscriberRos::ProxSensSubscriberRos(ros::NodeHandle *node)
{
    nodeHandle = node;
}

void ProxSensSubscriberRos::subscribe(ProxSens *proxSens)
{
    proxSensSubRos = nodeHandle->subscribe("gpioEdgeChange", 1000, &ProxSens::edgeChangeCallback, proxSens);
}

/* Services interface implementation */
ProxSensClientsRos::ProxSensClientsRos(ros::NodeHandle *node)
{
    gpioSetInputClientRos = node->serviceClient<hal_pigpio::hal_pigpioSetInputMode>("hal_pigpioSetInputMode");
    gpioSetOutputClientRos = node->serviceClient<hal_pigpio::hal_pigpioSetOutputMode>("hal_pigpioSetOutputMode");
    gpioSetCallbackClientRos = node->serviceClient<hal_pigpio::hal_pigpioSetCallback>("hal_pigpioSetCallback");
    gpioSendTriggerPulseClientRos = node->serviceClient<hal_pigpio::hal_pigpioSendTriggerPulse>("hal_pigpioSendTriggerPulse");
}

ros::ServiceClient ProxSensClientsRos::getSetInputClientHandle()
{
    return gpioSetInputClientRos;
}

ros::ServiceClient ProxSensClientsRos::getSetCallbackClientHandle()
{
    return gpioSetOutputClientRos;
}

ros::ServiceClient ProxSensClientsRos::getSetOutputClientHandle()
{
    return gpioSetCallbackClientRos;
}

ros::ServiceClient ProxSensClientsRos::getSendTriggerPulseClientHandle()
{
    return gpioSendTriggerPulseClientRos;
}

/* Proximity sensor implementation */
ProxSens::ProxSens(ProxSensSubscriber *proxSensSubscriber, ProxSensPublisher *proxSensPublisher, ProxSensClients *proxSensServiceClients) : edgeChangeType(NO_CHANGE),
                                                                                                                                            timestamp(0),
                                                                                                                                            echoCallbackId(0),
                                                                                                                                            distanceInCm(UINT16_MAX),
                                                                                                                                            proxSensPub(proxSensPublisher),
                                                                                                                                            proxSensClients(proxSensServiceClients)
{
    proxSensSubscriber->subscribe(this);
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

            distanceInCm = (uint16_t)((edgeLength) / 59.0);
        }
        else if (edgeChangeType == RISING_EDGE)
        {
            lastEdgeChangeType = edgeChangeType;
            lastTimestamp = timestamp;
        }
    }
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

    setCallbackSrv.request.gpioId = PROXSENS_ECHO_GPIO;
    setCallbackSrv.request.edgeChangeType = AS_EITHER_EDGE;

    setOutputModeSrv.request.gpioId = PROXSENS_TRIGGER_GPIO;

    proxSensClients->getSetInputClientHandle().call(setInputModeSrv);

    proxSensClients->getSetCallbackClientHandle().call(setCallbackSrv);
    if (setCallbackSrv.response.hasSucceeded)
    {
        echoCallbackId = setCallbackSrv.response.callbackId;
    }

    proxSensClients->getSetOutputClientHandle().call(setOutputModeSrv);
}

void ProxSens::trigger(void)
{
    hal_pigpio::hal_pigpioSendTriggerPulse sendTriggerPulseSrv;

    sendTriggerPulseSrv.request.gpioId = PROXSENS_TRIGGER_GPIO;
    sendTriggerPulseSrv.request.pulseLengthInUs = PROXSENS_TRIGGER_LENGTH_US;

    proxSensClients->getSendTriggerPulseClientHandle().call(sendTriggerPulseSrv);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hal_proxsens");
    ros::NodeHandle node;

    ProxSensPublisherRos proxSensPublisherRos = ProxSensPublisherRos(&node);
    ProxSensSubscriberRos proxSensSubscriber = ProxSensSubscriberRos(&node);
    ProxSensClientsRos proxSensServiceClients = ProxSensClientsRos(&node);

    ProxSens proxSens = ProxSens(&proxSensSubscriber, &proxSensPublisherRos, &proxSensServiceClients);
    proxSens.configureGpios();

    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        proxSens.publishMessage();
        proxSens.trigger();

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}