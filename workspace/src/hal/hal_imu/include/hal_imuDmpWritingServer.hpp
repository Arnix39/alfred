#ifndef HAL_IMU_DMP_WRITING_SERVER
#define HAL_IMU_DMP_WRITING_SERVER

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "hal_imuDmpMemory.hpp"
#include "hal_mpu6050.hpp"

// Services and messages headers (generated)
#include "hal_imu_interfaces/action/hal_imu_write_dmp.hpp"
#include "hal_imu_interfaces/srv/hal_imu_get_handle.hpp"
#include "hal_pigpio_interfaces/srv/hal_pigpio_i2c_write_byte_data.hpp"
#include "hal_pigpio_interfaces/srv/hal_pigpio_i2c_write_block_data.hpp"

class ImuDmpWritingServer
{
private:
    hal_imu::hal_imuWriteDmpFeedback feedback;
    hal_imu::hal_imuWriteDmpResult result;
    int32_t imuHandle;

public:
    ImuDmpWritingServer();
    ~ImuDmpWritingServer() = default;
    void getI2cHandle(void);
    void startServer(void);
    bool isI2cInitialised(void);
    void writeDmp(void);
    bool writeByteInRegister(uint8_t registerToWrite, uint8_t value);
    bool writeData(uint8_t bank, uint8_t addressInBank, std::vector<uint8_t> data);
    bool writeDataBlock(uint8_t chipRegister, std::vector<uint8_t> data);
    bool isNotStarted(void);
    void starts(void);
};

#endif