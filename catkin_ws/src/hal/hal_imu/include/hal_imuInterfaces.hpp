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

class ImuServersRos : public ImuServers
{
private:
    ros::ServiceServer imuGetHandleServerRos;
    ros::NodeHandle *nodeHandle;

public:
    ImuServersRos(ros::NodeHandle *node);
    ~ImuServersRos() = default;
    void advertiseGetHandleService(Imu *imu) override;
};

class ImuClientsRos : public ImuClients
{
private:
    ros::ServiceClient i2cOpenClientRos;
    ros::ServiceClient i2cCloseClientRos;
    ros::ServiceClient i2cReadByteDataClientRos;
    ros::ServiceClient i2cWriteByteDataClientRos;

public:
    ImuClientsRos(ros::NodeHandle *node);
    ~ImuClientsRos() = default;
    ros::ServiceClient *getI2cOpenHandle() override;
    ros::ServiceClient *getI2cCloseHandle() override;
    ros::ServiceClient *getReadByteDataClientHandle() override;
    ros::ServiceClient *getWriteByteDataClientHandle() override;
};

#endif