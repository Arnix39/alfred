#ifndef HAL_IMUDMPWRITINGSERVERINTERFACES
#define HAL_IMUDMPWRITINGSERVERINTERFACES

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
    ros::ServiceClient i2cWriteBlockDataClientRos;
    ros::ServiceClient i2cGetHandleClientRos;

public:
    ImuDmpWritingClientsRos(ros::NodeHandle *node);
    ~ImuDmpWritingClientsRos() = default;
    ros::ServiceClient *getReadByteDataClientHandle() override;
    ros::ServiceClient *getWriteByteDataClientHandle() override;
    ros::ServiceClient *getWriteBlockDataClientHandle() override;
    ros::ServiceClient *getGetHandleClientHandle() override;
};

#endif