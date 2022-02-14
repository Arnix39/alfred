#ifndef HAL_IMUDMPWRITINGSERVERVIRTUALS
#define HAL_IMUDMPWRITINGSERVERVIRTUALS

#include "ros/ros.h"

// Services and messages headers (generated)
#include "hal_imu/hal_imuWriteDmpAction.h"

typedef actionlib::SimpleActionServer<hal_imu::hal_imuWriteDmpAction> imuActionServer_t;

class ImuDmpWritingServer;

class ImuActionServer
{
public:
    ImuActionServer() {}
    virtual ~ImuActionServer() {}
    virtual void registerCallback(ImuDmpWritingServer *imuDmpWritingServer) = 0;
    virtual imuActionServer_t *getActionServerHandle() = 0;
};

class ImuClients
{
public:
    ImuClients() {}
    virtual ~ImuClients() {}
    virtual ros::ServiceClient *getReadByteDataClientHandle() = 0;
    virtual ros::ServiceClient *getWriteByteDataClientHandle() = 0;
};

#endif