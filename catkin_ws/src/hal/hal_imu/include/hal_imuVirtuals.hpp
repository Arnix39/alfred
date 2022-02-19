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

class ImuServers
{
public:
    ImuServers() {}
    virtual ~ImuServers() {}
    virtual void advertiseGetHandleService(Imu *imu) = 0;
};

class ImuClients
{
public:
    ImuClients() {}
    virtual ~ImuClients() {}
    virtual ros::ServiceClient *getI2cOpenHandle() = 0;
    virtual ros::ServiceClient *getI2cCloseHandle() = 0;
};

#endif