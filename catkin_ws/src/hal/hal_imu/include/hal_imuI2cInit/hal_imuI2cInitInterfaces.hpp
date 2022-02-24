#ifndef HAL_IMUI2CINITINTERFACES
#define HAL_IMUI2CINITINTERFACES

#include "hal_imuI2cInitVirtuals.hpp"

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

#endif