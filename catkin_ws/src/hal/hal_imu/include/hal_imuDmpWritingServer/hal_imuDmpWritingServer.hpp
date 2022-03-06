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
#include "hal_pigpio/hal_pigpioI2cWriteWordData.h"
#include "hal_pigpio/hal_pigpioI2cWriteBlockData.h"
#include "hal_imu/hal_imuI2cHeartbeatMsg.h"

class ImuDmpWritingServer
{
private:
    ImuDmpWritingClients *imuDmpClients;
    ImuDmpWritingActionServer *imuDmpWritingServer;
    ImuDmpWritingServerSubscribers *imuDmpWritingServerSubs;
    hal_imu::hal_imuWriteDmpFeedback feedback;
    hal_imu::hal_imuWriteDmpResult result;
    bool i2cInitialised;
    int32_t imuHandle;

public:
    ImuDmpWritingServer(ImuDmpWritingActionServer *imuWriteDmpServer, ImuDmpWritingClients *imuDmpServiceClients, ImuDmpWritingServerSubscribers *imuDmpWritingServerSubscribers);
    ~ImuDmpWritingServer() = default;
    void getI2cHandle(void);
    void startServer(void);
    bool getI2cInitialised(void);
    void imuDmpWritingServerI2cInitHeartbeatCallback(const hal_imu::hal_imuI2cHeartbeatMsg &msg);
    void writeDmp(void);
    bool writeByteInRegister(uint8_t registerToWrite, uint8_t value);
    bool writeWordInRegister(uint8_t registerToWrite, uint16_t value);
    bool writeDataInRegister(uint8_t registerToWrite, uint8_t value);
    bool writeData(uint8_t bank, uint8_t addressInBank, std::vector<uint8_t> data);
    bool writeDataBlock(uint8_t chipRegister, std::vector<uint8_t> data);
};

#endif