#ifndef HAL_IMUI2CINIT
#define HAL_IMUI2CINIT

#include "ros/ros.h"

#include "hal_imuMPU6050.hpp"
#include "hal_imuI2cInitVirtuals.hpp"

// Services and messages headers (generated)
#include "hal_pigpio/hal_pigpioI2cOpen.h"
#include "hal_pigpio/hal_pigpioI2cClose.h"
#include "hal_pigpio/hal_pigpioI2cReadByteData.h"
#include "hal_pigpio/hal_pigpioI2cWriteByteData.h"
#include "hal_pigpio/hal_pigpioHeartbeatMsg.h"
#include "hal_imu/hal_imuI2cHeartbeatMsg.h"
#include "hal_imu/hal_imuGetHandle.h"

#define IMU_I2C_BUS 0x1

class ImuI2cInit
{
private:
    ImuI2cInitClients *imuI2cInitClients;
    ImuI2cInitServers *imuI2cInitServers;
    ImuI2cInitPublisher *imuI2cInitPub;
    ImuI2cInitSubscribers *imuI2cInitSubs;
    int32_t imuHandle;
    bool pigpioNodeStarted;

public:
    ImuI2cInit(ImuI2cInitClients *imuI2cInitServiceClients, ImuI2cInitServers *imuI2cInitServiceServers, ImuI2cInitPublisher *imuI2cInitPublisher, ImuI2cInitSubscribers *imuI2cInitSubscribers);
    ~ImuI2cInit();
    bool getHandle(hal_imu::hal_imuGetHandle::Request &req,
                   hal_imu::hal_imuGetHandle::Response &res);
    void initI2cCommunication(void);
    bool writeBitInRegister(uint8_t registerToWrite, uint8_t bitToWrite, uint8_t valueOfBit);
    void setClockSource(void);
    void setSleepDisabled(void);
    bool getPigpioNodeStarted(void);
    void publishHeartbeat(void);
    void pigpioHeartbeatCallback(const hal_pigpio::hal_pigpioHeartbeatMsg &msg);
};

#endif