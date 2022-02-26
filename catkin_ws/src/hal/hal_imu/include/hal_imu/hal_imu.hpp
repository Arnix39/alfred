#ifndef HAL_IMU
#define HAL_IMU

#include "ros/ros.h"
#include <actionlib/client/simple_action_client.h>

#include "hal_imuVirtuals.hpp"

// Services and messages headers (generated)
#include "hal_pigpio/hal_pigpioI2cReadByteData.h"
#include "hal_pigpio/hal_pigpioI2cWriteByteData.h"
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

public:
    Imu(ImuPublisher *imuMessagePublisher, ImuClients *imuServiceClients);
    ~Imu() = default;
    void init(void);
    void writeDmp(void);
    void enable6AxisQuaternion(void);
    void calibrateAccelerometer(void);
    bool writeByteInRegister(uint8_t chipRegister, uint8_t value);
    void enableGyroCalibrationOnDMP(void);
    bool writeDataToDmp(uint16_t address, uint8_t size, const unsigned char *data);
};

#endif