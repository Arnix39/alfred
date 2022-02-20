#ifndef HAL_IMU
#define HAL_IMU

#include "ros/ros.h"
#include <actionlib/client/simple_action_client.h>

#include "hal_imuVirtuals.hpp"

// Services and messages headers (generated)
#include "hal_pigpio/hal_pigpioI2cOpen.h"
#include "hal_pigpio/hal_pigpioI2cClose.h"
#include "hal_pigpio/hal_pigpioI2cReadByteData.h"
#include "hal_pigpio/hal_pigpioI2cWriteByteData.h"
#include "hal_imu/hal_imuWriteDmpAction.h"
#include "hal_imu/hal_imuGetHandle.h"
#include "hal_imu/hal_imuMsg.h"

#define IMU_I2C_ADDRESS 0x68
#define IMU_I2C_BUS 0x0

/* Extracted from Motion Driver 5.1.3 from Invensense */
#define DMP_FEATURE_6X_LP_QUAT 0xA9E
#define DMP_FEATURE_6X_LP_QUAT_SIZE 4
static const unsigned char dmp6AxisQuaternion[DMP_FEATURE_6X_LP_QUAT_SIZE] = {0x20, 0x28, 0x30, 0x38};

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
    ~Imu();
    bool getHandle(hal_imu::hal_imuGetHandle::Request &req,
                   hal_imu::hal_imuGetHandle::Response &res);
    void init(void);
    void initI2cCommunication(void);
    void writeDmp(void);
    void calibrateAccelerometer(void);
    bool writeByteInRegister(uint8_t chipRegister, uint8_t value);
    void enableGyroCalibrationOnDMP(void);
};

#endif