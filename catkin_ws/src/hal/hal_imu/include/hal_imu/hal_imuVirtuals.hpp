#ifndef HAL_IMUVIRTUALS
#define HAL_IMUVIRTUALS

#include "ros/ros.h"

// Services and messages headers (generated)
#include "hal_imu/hal_imuMsg.h"

class Imu;

class ImuPublisher
{
public:
    ImuPublisher() {}
    virtual ~ImuPublisher() {}
    virtual void publish(hal_imu::hal_imuMsg message) = 0;
};

class ImuClients
{
public:
    ImuClients() {}
    virtual ~ImuClients() {}
    virtual ros::ServiceClient *getI2cOpenHandle() = 0;
    virtual ros::ServiceClient *getI2cCloseHandle() = 0;
    virtual ros::ServiceClient *getReadByteDataClientHandle() = 0;
    virtual ros::ServiceClient *getWriteByteDataClientHandle() = 0;
};

#endif