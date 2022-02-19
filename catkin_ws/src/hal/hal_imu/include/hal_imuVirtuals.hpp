#ifndef HAL_IMUVIRTUALS
#define HAL_IMUVIRTUALS

#include "ros/ros.h"

class Imu;

class ImuServers
{
public:
    ImuServers() {}
    virtual ~ImuServers() {}
    virtual void advertise(Imu *imu) = 0;
};

#endif