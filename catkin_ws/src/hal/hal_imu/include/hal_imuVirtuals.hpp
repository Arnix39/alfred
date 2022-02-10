#ifndef HAL_IMUVIRTUALS
#define HAL_IMUVIRTUALS

#include "ros/ros.h"

class ImuClients
{
public:
    ImuClients() {}
    virtual ~ImuClients() {}
    virtual ros::ServiceClient getReadByteDataClientHandle() = 0;
    virtual ros::ServiceClient getWriteByteDataClientHandle() = 0;
};

#endif