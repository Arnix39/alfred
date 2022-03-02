#ifndef HAL_IMUI2CINITVIRTUALS
#define HAL_IMUI2CINITVIRTUALS

#include "ros/ros.h"

// Services and messages headers (generated)

class ImuI2cInit;

class ImuI2cInitServers
{
public:
    ImuI2cInitServers() {}
    virtual ~ImuI2cInitServers() {}
    virtual void advertiseGetHandleService(ImuI2cInit *imuI2cInit) = 0;
};

class ImuI2cInitClients
{
public:
    ImuI2cInitClients() {}
    virtual ~ImuI2cInitClients() {}
    virtual ros::ServiceClient *getI2cOpenHandle() = 0;
    virtual ros::ServiceClient *getI2cCloseHandle() = 0;
    virtual ros::ServiceClient *getI2cReadByteDataClientHandle() = 0;
    virtual ros::ServiceClient *getI2cWriteByteDataClientHandle() = 0;
};

#endif