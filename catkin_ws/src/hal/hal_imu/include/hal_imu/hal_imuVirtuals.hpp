#ifndef HAL_IMU_VIRTUALS
#define HAL_IMU_VIRTUALS

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

class ImuSubscribers
{
public:
    ImuSubscribers() {}
    virtual ~ImuSubscribers() {}
    virtual void subscribe(Imu *imu) = 0;
};

class ImuClients
{
public:
    ImuClients() {}
    virtual ~ImuClients() {}
    virtual ros::ServiceClient *getReadByteDataClientHandle() = 0;
    virtual ros::ServiceClient *getWriteByteDataClientHandle() = 0;
    virtual ros::ServiceClient *getReadWordDataClientHandle() = 0;
    virtual ros::ServiceClient *getWriteWordDataClientHandle() = 0;
    virtual ros::ServiceClient *getReadBlockDataClientHandle() = 0;
    virtual ros::ServiceClient *getWriteBlockDataClientHandle() = 0;
    virtual ros::ServiceClient *getGetHandleClientHandle() = 0;
};

#endif