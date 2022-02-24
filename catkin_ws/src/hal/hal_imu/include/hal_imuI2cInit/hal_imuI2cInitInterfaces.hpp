#ifndef HAL_IMUI2CINITINTERFACES
#define HAL_IMUI2CINITINTERFACES

#include "hal_imuI2cInitVirtuals.hpp"

class ImuI2cInitServersRos : public ImuI2cInitServers
{
private:
    ros::ServiceServer imuGetHandleServerRos;
    ros::NodeHandle *nodeHandle;

public:
    ImuI2cInitServersRos(ros::NodeHandle *node);
    ~ImuI2cInitServersRos() = default;
    void advertiseGetHandleService(Imu *imu) override;
};

class ImuI2cInitClientsRos : public ImuI2cInitClients
{
private:
    ros::ServiceClient i2cOpenClientRos;
    ros::ServiceClient i2cCloseClientRos;

public:
    ImuI2cInitClientsRos(ros::NodeHandle *node);
    ~ImuI2cInitClientsRos() = default;
    ros::ServiceClient *getI2cOpenHandle() override;
    ros::ServiceClient *getI2cCloseHandle() override;
};

#endif