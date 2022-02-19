#ifndef HAL_IMUINTERFACES
#define HAL_IMUINTERFACES

#include "hal_imuVirtuals.hpp"

class ImuServersRos : public ImuServers
{
private:
    ros::ServiceServer imuGetHandleServerRos;
    ros::NodeHandle *nodeHandle;

public:
    ImuServersRos(ros::NodeHandle *node);
    ~ImuServersRos() = default;
    void advertise(Imu *imu) override;
};

#endif