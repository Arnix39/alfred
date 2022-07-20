#ifndef HAL_IMU
#define HAL_IMU

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

// Services and messages headers (generated)
#include "hal_pigpio_interfaces/srv/hal_pigpio_i2c_read_byte_data.hpp"
#include "hal_pigpio_interfaces/srv/hal_pigpio_i2c_write_byte_data.hpp"
#include "hal_pigpio_interfaces/srv/hal_pigpio_i2c_write_block_data.hpp"
#include "hal_pigpio_interfaces/srv/hal_pigpio_i2c_imu_reading.hpp"
#include "hal_imu_interfaces/srv/hal_imu_get_handle.hpp"
#include "hal_imu_interfaces/msg/hal_imu.hpp"
#include "hal_imu_interfaces/action/hal_imu_write_dmp.hpp"

#define IMU_GYROSCOPE_X_OFFSET 0
#define IMU_GYROSCOPE_Y_OFFSET -34
#define IMU_GYROSCOPE_Z_OFFSET -9

#define IMU_ACCELEROMETER_X_OFFSET -556
#define IMU_ACCELEROMETER_Y_OFFSET -1188
#define IMU_ACCELEROMETER_Z_OFFSET 873

typedef actionlib::SimpleActionClient<hal_imu::hal_imuWriteDmpAction> imuActionClient_t;

struct SensorBias
{
    const char axis;
    const int16_t bias;
    const uint8_t msbRegister;
    const uint8_t lsbRegister;
};

class Imu : public rclcpp_lifecycle::LifecycleNode
{
private:
    int32_t imuHandle;
    bool i2cInitialised;

public:
    Imu();
    ~Imu() = default;
    void getI2cHandle(void);
    void init(void);
    void writeDmp(void);
    void setDmpRate(uint16_t rate);
    void setMpuRate(uint16_t rate);
    void enableDmp(void);
    void configureDmpFeatures(void);
    void resetImu(void);
    void resetFifo(void);
    void setClockSource(void);
    bool writeDataToDmp(uint8_t bank, uint8_t addressInBank, std::vector<uint8_t> data);
    void startImuReading(void);
    void stopImuReading(void);
    void setConfiguration(void);
    void setGyroscopeSensitivity(void);
    void setAccelerometerSensitivity(void);
    void setAccelerometerOffsets(void);
    void setGyroscopeOffsets(void);
    bool writeSensorBiases(const std::vector<SensorBias> sensorBiases);
    int16_t readByteFromRegister(uint8_t registerToRead);
    bool writeBitInRegister(uint8_t registerToWrite, uint8_t bitToWrite, uint8_t valueOfBit);
    bool writeByteInRegister(uint8_t registerToWrite, uint8_t value);
    bool writeDataBlock(uint8_t registerToWrite, std::vector<uint8_t> data);
};

#endif