#ifndef HAL_IMUDMPWRITINGSERVER
#define HAL_IMUDMPWRITINGSERVER

#include "ros/ros.h"
#include <actionlib/server/simple_action_server.h>

#include "hal_imuDmpMemory.hpp"
#include "hal_imuDmpWritingServerVirtuals.hpp"

// Services and messages headers (generated)
#include "hal_imu/hal_imuWriteDmpAction.h"
#include "hal_imu/hal_imuWriteDmpResult.h"
#include "hal_imu/hal_imuWriteDmpGoal.h"
#include "hal_imu/hal_imuGetHandle.h"
#include "hal_pigpio/hal_pigpioI2cReadByteData.h"
#include "hal_pigpio/hal_pigpioI2cWriteByteData.h"

class ImuDmpWritingServer
{
private:
    ImuClients *imuClients;
    ImuActionServer *imuDmpWritingServer;
    hal_imu::hal_imuWriteDmpFeedback feedback;
    hal_imu::hal_imuWriteDmpResult result;
    int32_t imuHandle;

public:
    ImuDmpWritingServer(ImuActionServer *imuWriteDmpServer, ImuClients *imuServiceClients);
    ~ImuDmpWritingServer() = default;
    void writeDmp(void);
    bool writeByteInRegister(uint8_t chipRegister, uint8_t value);
    bool writeByte(uint8_t bank, uint8_t addressInBank, uint8_t value);
};

#endif