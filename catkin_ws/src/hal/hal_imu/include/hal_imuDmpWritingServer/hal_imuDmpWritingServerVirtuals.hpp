#ifndef HAL_IMUDMPWRITINGSERVERVIRTUALS
#define HAL_IMUDMPWRITINGSERVERVIRTUALS

#include "ros/ros.h"

// Services and messages headers (generated)
#include "hal_imu/hal_imuWriteDmpAction.h"

typedef actionlib::SimpleActionServer<hal_imu::hal_imuWriteDmpAction> imuDmpWritingActionServer_t;

class ImuDmpWritingServer;

class ImuDmpWritingActionServer
{
public:
    ImuDmpWritingActionServer() {}
    virtual ~ImuDmpWritingActionServer() {}
    virtual void registerCallback(ImuDmpWritingServer *imuDmpWritingServer) = 0;
    virtual imuDmpWritingActionServer_t *getActionServerHandle() = 0;
};

class ImuDmpWritingClients
{
public:
    ImuDmpWritingClients() {}
    virtual ~ImuDmpWritingClients() {}
    virtual ros::ServiceClient *getReadByteDataClientHandle() = 0;
    virtual ros::ServiceClient *getWriteByteDataClientHandle() = 0;
    virtual ros::ServiceClient *getWriteBlockDataClientHandle() = 0;
    virtual ros::ServiceClient *getGetHandleClientHandle() = 0;
};

#endif