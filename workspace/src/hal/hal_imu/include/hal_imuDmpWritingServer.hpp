#ifndef HAL_IMU_DMP_WRITING_SERVER
#define HAL_IMU_DMP_WRITING_SERVER

#include <thread>

#include "rclcpp_action/rclcpp_action.hpp"

#include "hal_imuDmpMemory.hpp"
#include "common.hpp"
#include "mpu6050.hpp"
#include "hal_i2cRegistersServices.hpp"

// Services and messages headers (generated)
#include "hal_pigpio_interfaces/srv/hal_pigpio_i2c_write_byte_data.hpp"
#include "hal_pigpio_interfaces/srv/hal_pigpio_i2c_write_block_data.hpp"
#include "hal_imu_interfaces/action/hal_imu_write_dmp.hpp"
#include "hal_imu_interfaces/srv/hal_imu_get_handle.hpp"

using HalImuWriteDmpAction = hal_imu_interfaces::action::HalImuWriteDmp;
using HalImuWriteDmpGoal = rclcpp_action::ServerGoalHandle<HalImuWriteDmpAction>;

class ImuDmpWritingServer : public rclcpp_lifecycle::LifecycleNode 
{
private:
    int32_t imuHandle;

    rclcpp::Client<hal_pigpio_interfaces::srv::HalPigpioI2cWriteByteData>::SharedPtr i2cWriteByteDataClient;
    rclcpp::Client<hal_pigpio_interfaces::srv::HalPigpioI2cWriteBlockData>::SharedPtr i2cWriteBlockDataClient;

    rclcpp_action::Server<HalImuWriteDmpAction>::SharedPtr imuDmpWritingServer;
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const HalImuWriteDmpAction::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<HalImuWriteDmpGoal> goal_handle);
    void handle_accepted(const std::shared_ptr<HalImuWriteDmpGoal> goal_handle);
    
public:
    ImuDmpWritingServer();
    ~ImuDmpWritingServer() = default;

    LifecycleCallbackReturn_t on_configure(const rclcpp_lifecycle::State & previous_state);
    LifecycleCallbackReturn_t on_activate(const rclcpp_lifecycle::State & previous_state);
    LifecycleCallbackReturn_t on_deactivate(const rclcpp_lifecycle::State & previous_state);
    LifecycleCallbackReturn_t on_cleanup(const rclcpp_lifecycle::State & previous_state);
    LifecycleCallbackReturn_t on_shutdown(const rclcpp_lifecycle::State & previous_state);
    LifecycleCallbackReturn_t on_error(const rclcpp_lifecycle::State & previous_state);
    
    void startServer(void);
    void writeDmp(const std::shared_ptr<HalImuWriteDmpGoal> goal_handle);
    bool writeData(uint8_t bank, uint8_t addressInBank, std::vector<uint8_t> data);
};

#endif