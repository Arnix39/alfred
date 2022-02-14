#ifndef HAL_IMUINTERFACES
#define HAL_IMUINTERFACES

#include "hal_imuVirtuals.hpp"

class ImuActionServerRos : public ImuActionServer
{
private:
    ros::NodeHandle *nodeHandle;
    imuActionServer_t imuWriteDmpServerRos;

public:
    ImuActionServerRos(ros::NodeHandle *node);
    ~ImuActionServerRos() = default;
    void registerCallback(ImuDmpWritingServer *imuDmpWritingServer) override;
    imuActionServer_t *getActionServerHandle() override;
};

class ImuClientsRos : public ImuClients
{
private:
    ros::ServiceClient i2cReadByteDataClientRos;
    ros::ServiceClient i2cWriteByteDataClientRos;

public:
    ImuClientsRos(ros::NodeHandle *node);
    ~ImuClientsRos() = default;
    ros::ServiceClient *getReadByteDataClientHandle() override;
    ros::ServiceClient *getWriteByteDataClientHandle() override;
};

#endif