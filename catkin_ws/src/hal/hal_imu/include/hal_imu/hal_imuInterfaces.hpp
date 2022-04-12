#ifndef HAL_IMU_INTERFACES
#define HAL_IMU_INTERFACES

#include "hal_imuVirtuals.hpp"

class ImuSubscribersRos : public ImuSubscribers
{
private:
    ros::Subscriber imuImuI2cInitHBSubRos;
    ros::NodeHandle *nodeHandle;

public:
    ImuSubscribersRos(ros::NodeHandle *node);
    ~ImuSubscribersRos() = default;
    void subscribe(Imu *imu) override;
};

class ImuClientsRos : public ImuClients
{
private:
    ros::ServiceClient i2cReadByteDataClientRos;
    ros::ServiceClient i2cWriteByteDataClientRos;
    ros::ServiceClient i2cReadWordDataClientRos;
    ros::ServiceClient i2cWriteWordDataClientRos;
    ros::ServiceClient i2cReadBlockDataClientRos;
    ros::ServiceClient i2cWriteBlockDataClientRos;
    ros::ServiceClient i2cGetHandleClientRos;
    ros::ServiceClient i2cImuReadingClientRos;

public:
    ImuClientsRos(ros::NodeHandle *node);
    ~ImuClientsRos() = default;
    ros::ServiceClient *getReadByteDataClientHandle() override;
    ros::ServiceClient *getWriteByteDataClientHandle() override;
    ros::ServiceClient *getReadWordDataClientHandle() override;
    ros::ServiceClient *getWriteWordDataClientHandle() override;
    ros::ServiceClient *getReadBlockDataClientHandle() override;
    ros::ServiceClient *getWriteBlockDataClientHandle() override;
    ros::ServiceClient *getGetHandleClientHandle() override;
    ros::ServiceClient *getImuReadingClientHandle() override;
};

#endif