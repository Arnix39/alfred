#ifndef HAL_IMU
#define HAL_IMU

#include "ros/ros.h"
#include <actionlib/client/simple_action_client.h>

#include "hal_imuVirtuals.hpp"

// Services and messages headers (generated)
#include "hal_pigpio/hal_pigpioI2cOpen.h"
#include "hal_imu/hal_imuWriteDmpAction.h"
#include "hal_imu/hal_imuGetHandle.h"
#include "hal_imu/hal_imuMsg.h"

typedef actionlib::SimpleActionClient<hal_imu::hal_imuWriteDmpAction> imuActionClient_t;

class Imu
{
private:
    ImuServers *imuServers;
    ImuClients *imuClients;
    ImuPublisher *imuPublisher;
    int32_t imuHandle;

public:
    Imu(ImuPublisher *imuMessagePublisher, ImuServers *imuServiceServers, ImuClients *imuServiceClients);
    bool getHandle(hal_imu::hal_imuGetHandle::Request &req,
                   hal_imu::hal_imuGetHandle::Response &res);
};

#endif