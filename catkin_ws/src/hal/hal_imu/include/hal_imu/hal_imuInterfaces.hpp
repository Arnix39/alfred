#ifndef HAL_IMUINTERFACES
#define HAL_IMUINTERFACES

#include "hal_imuVirtuals.hpp"

class ImuPublisherRos : public ImuPublisher
{
private:
    ros::Publisher imuPublisherRos;

public:
    ImuPublisherRos(ros::NodeHandle *node);
    ~ImuPublisherRos() = default;
    void publish(hal_imu::hal_imuMsg message) override;
};

class ImuClientsRos : public ImuClients
{
private:
    ros::ServiceClient i2cReadByteDataClientRos;
    ros::ServiceClient i2cWriteByteDataClientRos;
    ros::ServiceClient i2cReadWordDataClientRos;
    ros::ServiceClient i2cWriteWordDataClientRos;
    ros::ServiceClient i2cWriteBlockDataClientRos;
    ros::ServiceClient i2cGetHandleClientRos;

public:
    ImuClientsRos(ros::NodeHandle *node);
    ~ImuClientsRos() = default;
    ros::ServiceClient *getReadByteDataClientHandle() override;
    ros::ServiceClient *getWriteByteDataClientHandle() override;
    ros::ServiceClient *getReadWordDataClientHandle() override;
    ros::ServiceClient *getWriteWordDataClientHandle() override;
    ros::ServiceClient *getWriteBlockDataClientHandle() override;
    ros::ServiceClient *getGetHandleClientHandle() override;
};

#endif