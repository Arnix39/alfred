#include "hal_proxSens.hpp"

ProxSens::ProxSens(ros::NodeHandle *node)
{
    proxSensPub = node->advertise<hal_proxsens::hal_proxsensMsg>("proxSensorValue", 1000);
    edgeChangeSub = node->subscribe("gpioEdgeChange", 1000, &ProxSens::edgeChangeCallback, this);

    gpioSetInputClient = node->serviceClient<hal_pigpio::hal_pigpioSetInputMode>("hal_pigpioSetInputMode");
    gpioSetOutputClient = node->serviceClient<hal_pigpio::hal_pigpioSetOutputMode>("hal_pigpioSetOutputMode");
}

void ProxSens::edgeChangeCallback(const hal_pigpio::hal_pigpioEdgeChangeMsg& msg)
{
    //TODO: filter out messages that are not for the proximity sensor GPIOs
    gpio = msg.gpioId;
    edgeChangeType = msg.edgeChangeType;
    timestamp = msg.timeSinceBoot_us;
}

void ProxSens::publishMessage(void)
{
    hal_proxsens::hal_proxsensMsg message; 

    //TODO: compute distanceInCm

    message.distanceInCm = distanceInCm;
    proxSensPub.publish(message);
}

void ProxSens::configureGpios(void)
{
    hal_pigpio::hal_pigpioSetInputMode setInputModeSrv;
    hal_pigpio::hal_pigpioSetCallback setCallbackSrv;
    hal_pigpio::hal_pigpioSetOutputMode setOutputModeSrv;

    setInputModeSrv.request.gpioId = PROXSENS_ECHO_GPIO;
    
    setCallbackSrv.request.gpioId = PROXSENS_ECHO_GPIO;
    setCallbackSrv.request.edgeChangeType = EITHER_EDGE;
    
    setOutputModeSrv.request.gpioId = PROXSENS_TRIGGER_GPIO;

    gpioSetInputClient.call(setInputModeSrv);

    gpioSetCallbackClient.call(setCallbackSrv);
    if(setCallbackSrv.response.hasSucceeded)
    {
        echoCallbackId = setCallbackSrv.response.callbackId;
    }

    gpioSetOutputClient.call(setOutputModeSrv);
}

void ProxSens::trigger(void)
{
    hal_pigpio::hal_pigpioSendTriggerPulse sendTriggerPulseSrv;

    sendTriggerPulseSrv.request.gpioId = PROXSENS_TRIGGER_GPIO;
    sendTriggerPulseSrv.request.pulseLengthInUs = PROXSENS_TRIGGER_LENGTH_US;
    
    gpioSendTriggerPulseClient.call(sendTriggerPulseSrv);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hal_proxsens");
    ros::NodeHandle node;

    ProxSens proxSens = ProxSens(&node);
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