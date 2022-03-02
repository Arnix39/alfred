#ifndef HAL_IMU
#define HAL_IMU

#include "ros/ros.h"
#include <actionlib/client/simple_action_client.h>

#include "hal_imuVirtuals.hpp"

// Services and messages headers (generated)
#include "hal_pigpio/hal_pigpioI2cReadByteData.h"
#include "hal_pigpio/hal_pigpioI2cWriteByteData.h"
#include "hal_pigpio/hal_pigpioI2cReadWordData.h"
#include "hal_pigpio/hal_pigpioI2cWriteWordData.h"
#include "hal_pigpio/hal_pigpioI2cWriteBlockData.h"
#include "hal_imu/hal_imuGetHandle.h"
#include "hal_imu/hal_imuWriteDmpAction.h"
#include "hal_imu/hal_imuMsg.h"

typedef actionlib::SimpleActionClient<hal_imu::hal_imuWriteDmpAction> imuActionClient_t;

class Imu
{
private:
    ImuClients *imuClients;
    ImuPublisher *imuPublisher;
    int32_t imuHandle;
    int16_t angle;

public:
    Imu(ImuPublisher *imuMessagePublisher, ImuClients *imuServiceClients);
    ~Imu() = default;
    void init(void);
    void writeDmp(void);
    void enableDmp(void);
    void configureDmpFeatures(void);
    void calibrateAccelerometer(void);
    bool writeByteInRegister(uint8_t registerToWrite, uint8_t value);
    bool writeWordInRegister(uint8_t registerToWrite, uint16_t value);
    bool writeDataBlock(uint8_t registerToWrite, std::vector<uint8_t> data);
    int16_t readByteFromRegister(uint8_t registerToRead);
    int32_t readWordFromRegister(uint8_t registerToRead);
    bool writeBitInRegister(uint8_t registerToWrite, uint8_t bitToWrite, uint8_t valueOfBit);
    void resetImu(void);
    void resetFifo(void);
    void writeOrientationMatrix(void);
    void setClockSource(void);
    void setSleepDisabled(void);
    bool writeDataToDmp(uint16_t address, std::vector<uint8_t> data);
    void publishMessage(void);
    void readMpuData(void);
};

#endif