#ifndef HAL_PROXSENS
#define HAL_PROXSENS

#include "ros/ros.h"

// Services and messages headers (generated)
#include "hal_pigpio/hal_pigpioSetInputMode.h"
#include "hal_pigpio/hal_pigpioSetOutputMode.h"
#include "hal_pigpio/hal_pigpioSetGpioHigh.h"
#include "hal_pigpio/hal_pigpioSetGpioLow.h"
#include "hal_pigpio/hal_pigpioSetCallback.h"
#include "hal_pigpio/hal_pigpioEdgeChangeMsg.h"
#include "hal_proxsens/hal_proxsensMsg.h"

#define PROXSENS_TRIGGER_GPIO 5
#define PROXSENS_ECHO_GPIO 6

#define RISING_EDGE     0
#define FALLING_EDGE    1
#define EITHER_EDGE     2

class ProxSens
{
private:
    ros::Subscriber edgeChangeSub;
    ros::Publisher proxSensPub;
    ros::ServiceClient gpioSetInputClient;
    ros::ServiceClient gpioSetCallbackClient;
    ros::ServiceClient gpioSetOutputClient;
    uint8_t gpio;
    uint8_t edgeChangeType;
    uint32_t timestamp;
    uint32_t echoCallbackId;
    uint16_t distanceInCm;
    void edgeChangeCallback(const hal_pigpio::hal_pigpioEdgeChangeMsg& msg);

public:
    ProxSens(ros::NodeHandle *node);
    void publishMessage(void);
    void configureGpios(void);
    void trigger(void);
};

#endif