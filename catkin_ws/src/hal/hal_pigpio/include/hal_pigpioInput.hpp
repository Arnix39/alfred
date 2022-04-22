#ifndef HAL_PIGPIO_INPUT
#define HAL_PIGPIO_INPUT

#include "ros/ros.h"

// Pigpio library
#include <pigpiod_if2.h>

// Services and messages headers (generated)
#include "hal_pigpio/hal_pigpioReadGpio.h"
#include "hal_pigpio/hal_pigpioSetCallback.h"
#include "hal_pigpio/hal_pigpioSetEncoderCallback.h"
#include "hal_pigpio/hal_pigpioEdgeChangeMsg.h"
#include "hal_pigpio/hal_pigpioEncoderCountMsg.h"

struct Motor
{
    uint8_t id;
    std::vector<unsigned> gpios;
    uint32_t encoderCount;
};

class PigpioInput
{
private:
    ros::Publisher gpioEdgeChangePub;
    ros::Publisher gpioEncoderCountPub;
    ros::ServiceServer readGpioService;
    ros::ServiceServer setCallbackService;
    ros::ServiceServer setEncoderCallbackService;
    int pigpioHandle;
    std::vector<uint> callbackList;
    std::vector<Motor> motors;
    inline void gpioEdgeChangeCallback(int handle, unsigned gpioId, unsigned edgeChangeType, uint32_t timeSinceBoot_us);
    static void c_gpioEdgeChangeCallback(int handle, unsigned gpioId, unsigned edgeChangeType, uint32_t timeSinceBoot_us, void *userData);
    inline void gpioEncoderEdgeChangeCallback(int handle, unsigned gpioId, unsigned edgeChangeType, uint32_t timeSinceBoot_us);
    static void c_gpioEncoderEdgeChangeCallback(int handle, unsigned gpioId, unsigned edgeChangeType, uint32_t timeSinceBoot_us, void *userData);

public:
    PigpioInput(ros::NodeHandle *node, int pigpioHandle);
    ~PigpioInput();
    bool readGpio(hal_pigpio::hal_pigpioReadGpio::Request &req,
                  hal_pigpio::hal_pigpioReadGpio::Response &res);
    bool setCallback(hal_pigpio::hal_pigpioSetCallback::Request &req,
                     hal_pigpio::hal_pigpioSetCallback::Response &res);
    bool setEncoderCallback(hal_pigpio::hal_pigpioSetEncoderCallback::Request &req,
                            hal_pigpio::hal_pigpioSetEncoderCallback::Response &res);
    void publishEncoderCount(const ros::TimerEvent &timerEvent);
};

#endif