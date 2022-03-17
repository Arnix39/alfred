#ifndef HAL_IMU_DMP_WRITING_SERVER_INTERFACES
#define HAL_IMU_DMP_WRITING_SERVER_INTERFACES

#include "hal_imuDmpWritingServerVirtuals.hpp"

class ImuDmpWritingActionServerRos : public ImuDmpWritingActionServer
{
private:
    imuDmpWritingActionServer_t imuWriteDmpServerRos;

public:
    ImuDmpWritingActionServerRos(ros::NodeHandle *node);
    ~ImuDmpWritingActionServerRos() = default;
    void registerCallback(ImuDmpWritingServer *imuDmpWritingServer) override;
    imuDmpWritingActionServer_t *getActionServerHandle() override;
};

class ImuDmpWritingClientsRos : public ImuDmpWritingClients
{
private:
    ros::ServiceClient i2cReadByteDataClientRos;
    ros::ServiceClient i2cWriteByteDataClientRos;
    ros::ServiceClient i2cWriteWordDataClientRos;
    ros::ServiceClient i2cWriteBlockDataClientRos;
    ros::ServiceClient i2cGetHandleClientRos;

public:
    ImuDmpWritingClientsRos(ros::NodeHandle *node);
    ~ImuDmpWritingClientsRos() = default;
    ros::ServiceClient *getReadByteDataClientHandle() override;
    ros::ServiceClient *getWriteByteDataClientHandle() override;
    ros::ServiceClient *getWriteWordDataClientHandle() override;
    ros::ServiceClient *getWriteBlockDataClientHandle() override;
    ros::ServiceClient *getGetHandleClientHandle() override;
};

class ImuDmpWritingServerSubscribersRos : public ImuDmpWritingServerSubscribers
{
private:
    ros::Subscriber imuDmpWritingServerImuI2cInitHBSubRos;
    ros::NodeHandle *nodeHandle;

public:
    ImuDmpWritingServerSubscribersRos(ros::NodeHandle *node);
    ~ImuDmpWritingServerSubscribersRos() = default;
    void subscribe(ImuDmpWritingServer *imuDmpWritingServer) override;
};

#endif