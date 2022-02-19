#ifndef HAL_IMU
#define HAL_IMU

#include "ros/ros.h"
#include <actionlib/client/simple_action_client.h>

// Services and messages headers (generated)
#include "hal_imu/hal_imuWriteDmpAction.h"
#include "hal_imu/hal_imuGetHandle.h"

typedef actionlib::SimpleActionClient<hal_imu::hal_imuWriteDmpAction> imuActionClient_t;

class Imu
{
private:
    int32_t imuHandle;

public:
    Imu();
    bool getHandle(hal_imu::hal_imuGetHandle::Request &req,
                   hal_imu::hal_imuGetHandle::Response &res);
};

#endif