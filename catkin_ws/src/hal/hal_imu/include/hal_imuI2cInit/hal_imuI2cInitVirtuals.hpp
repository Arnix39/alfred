#ifndef HAL_IMUI2CINITVIRTUALS
#define HAL_IMUI2CINITVIRTUALS

#include "ros/ros.h"

// Services and messages headers (generated)

class ImuServers
{
public:
    ImuServers() {}
    virtual ~ImuServers() {}
    virtual void advertiseGetHandleService(Imu *imu) = 0;
};

#endif