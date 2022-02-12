#ifndef HAL_IMUDMPWRITINGSERVER
#define HAL_IMUDMPWRITINGSERVER

#include "ros/ros.h"
#include <actionlib/server/simple_action_server.h>

#include "hal_imuDmpMemory.hpp"
#include "hal_imuVirtuals.hpp"

// Services and messages headers (generated)
#include "hal_imu/hal_imuWriteDmpAction.h"
#include "hal_imu/hal_imuWriteDmpResult.h"
#include "hal_imu/hal_imuWriteDmpGoal.h"
#include "hal_pigpio/hal_pigpioI2cReadByteData.h"
#include "hal_pigpio/hal_pigpioI2cWriteByteData.h"

class ImuDmpWritingServer
{
private:
    ros::NodeHandle *nodeHandle;
    ImuClients *imuClients;
    actionlib::SimpleActionServer<hal_imu::hal_imuWriteDmpAction> imuDmpWritingServer;
    hal_imu::hal_imuWriteDmpFeedback feedback;
    hal_imu::hal_imuWriteDmpResult result;
    int32_t imuHandle;

public:
    ImuDmpWritingServer(ros::NodeHandle *node, ImuClients *imuServiceClients);
    ~ImuDmpWritingServer() = default;
    void writeDmp(const hal_imu::hal_imuWriteDmpGoalConstPtr &goal);
    bool writeByteInRegister(uint8_t chipRegister, uint8_t value);
    bool writeByte(uint8_t bank, uint8_t addressInBank, uint8_t value);
};

#endif