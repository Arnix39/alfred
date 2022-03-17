#ifndef HAL_PROXSENS
#define HAL_PROXSENS

#include "stdint.h"

#include "ros/ros.h"
#include "hal_proxSensVirtuals.hpp"

// Services and messages headers (generated)
#include "hal_pigpio/hal_pigpioSetInputMode.h"
#include "hal_pigpio/hal_pigpioSetOutputMode.h"
#include "hal_pigpio/hal_pigpioSetGpioHigh.h"
#include "hal_pigpio/hal_pigpioSendTriggerPulse.h"
#include "hal_pigpio/hal_pigpioSetCallback.h"
#include "hal_pigpio/hal_pigpioEdgeChangeMsg.h"
#include "hal_pigpio/hal_pigpioHeartbeatMsg.h"
#include "hal_proxsens/hal_proxsensMsg.h"

#define PROXSENS_TRIGGER_GPIO 5
#define PROXSENS_ECHO_GPIO 6
#define PROXSENS_LEVEL_SHIFTER_OE_GPIO 10
#define PROXSENS_TRIGGER_LENGTH_US 20

#define AS_RISING_EDGE 0
#define AS_FALLING_EDGE 1
#define AS_EITHER_EDGE 2

#define FALLING_EDGE 0
#define RISING_EDGE 1
#define NO_CHANGE 2

class ProxSens
{
private:
    ProxSensPublisher *proxSensPub;
    ProxSensClients *proxSensClients;
    ProxSensSubscriber *proxSensSub;
    uint8_t edgeChangeType;
    uint32_t timestamp;
    uint32_t echoCallbackId;
    uint16_t distanceInCm;
    bool pigpioNodeStarted;
    bool isStarted;

public:
    ProxSens(ProxSensSubscriber *proxSensSubscriber, ProxSensPublisher *proxSensPub, ProxSensClients *proxSensServiceClients);
    ~ProxSens() = default;
    void publishMessage(void);
    void configureGpios(void);
    void trigger(void);
    void enableOutputLevelShifter(void);
    void edgeChangeCallback(const hal_pigpio::hal_pigpioEdgeChangeMsg &msg);
    void pigpioHeartbeatCallback(const hal_pigpio::hal_pigpioHeartbeatMsg &msg);
    bool isPigpioNodeStarted(void);
    bool isNotStarted(void);
    void starts(void);
    void publishAndGetDistance(const ros::TimerEvent &timerEvent);
};

#endif