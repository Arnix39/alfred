#ifndef HAL_PIGPIO_INIT
#define HAL_PIGPIO_INIT

#include "ros/ros.h"

// Pigpio library
#include <pigpiod_if2.h>

// Services and messages headers (generated)
#include "hal_pigpio/hal_pigpioGetHandle.h"
#include "hal_pigpio/hal_pigpioGetMode.h"
#include "hal_pigpio/hal_pigpioSetInputMode.h"
#include "hal_pigpio/hal_pigpioSetOutputMode.h"
#include "hal_pigpio/hal_pigpioSetPullUp.h"
#include "hal_pigpio/hal_pigpioSetPullDown.h"
#include "hal_pigpio/hal_pigpioClearResistor.h"
#include "hal_pigpio/hal_pigpioHeartbeatMsg.h"

class PigpioInit
{
private:
    ros::ServiceServer getHandleService;
    ros::ServiceServer getModeService;
    ros::ServiceServer setInputModeService;
    ros::ServiceServer setOutputModeService;
    ros::ServiceServer setPullUpService;
    ros::ServiceServer setPullDownService;
    ros::ServiceServer clearResistorService;
    ros::Publisher heartbeatPublisher;
    int pigpioHandle;

public:
    PigpioInit(ros::NodeHandle *node, int pigpioHandle);
    ~PigpioInit();
    bool getHandle(hal_pigpio::hal_pigpioGetHandle::Request &req,
                   hal_pigpio::hal_pigpioGetHandle::Response &res);
    bool getMode(hal_pigpio::hal_pigpioGetMode::Request &req,
                 hal_pigpio::hal_pigpioGetMode::Response &res);
    bool setInputMode(hal_pigpio::hal_pigpioSetInputMode::Request &req,
                      hal_pigpio::hal_pigpioSetInputMode::Response &res);
    bool setOutputMode(hal_pigpio::hal_pigpioSetOutputMode::Request &req,
                       hal_pigpio::hal_pigpioSetOutputMode::Response &res);
    bool setPullUp(hal_pigpio::hal_pigpioSetPullUp::Request &req,
                   hal_pigpio::hal_pigpioSetPullUp::Response &res);
    bool setPullDown(hal_pigpio::hal_pigpioSetPullDown::Request &req,
                     hal_pigpio::hal_pigpioSetPullDown::Response &res);
    bool clearResistor(hal_pigpio::hal_pigpioClearResistor::Request &req,
                       hal_pigpio::hal_pigpioClearResistor::Response &res);
    void publishHeartbeat(const ros::TimerEvent &timerEvent);
};

#endif