#ifndef HAL_IMUDMPWRITINGSERVER
#define HAL_IMUDMPWRITINGSERVER

#include "ros/ros.h"
#include <actionlib/server/simple_action_server.h>

// Services and messages headers (generated)
#include "hal_imu/hal_imuWriteDmpAction.h"
#include "hal_imu/hal_imuWriteDmpResult.h"
#include "hal_imu/hal_imuWriteDmpGoal.h"

class ImuDmpWritingServer
{
private:
    ros::NodeHandle *nodeHandle;
    actionlib::SimpleActionServer<hal_imu::hal_imuWriteDmpAction> imuDmpWritingServer;
    hal_imu::hal_imuWriteDmpFeedback feedback;
    hal_imu::hal_imuWriteDmpResult result;

public:
    ImuDmpWritingServer(ros::NodeHandle *node);
    ~ImuDmpWritingServer() = default;
    void writeDmp(const hal_imu::hal_imuWriteDmpGoalConstPtr &goal);
};

#endif