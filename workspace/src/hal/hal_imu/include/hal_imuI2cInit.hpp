#ifndef HAL_IMU_I2C_INIT
#define HAL_IMU_I2C_INIT

#include "common.hpp"
#include "mpu6050.hpp"

// Services and messages headers (generated)
#include "hal_pigpio_interfaces/srv/hal_pigpio_i2c_open.hpp"
#include "hal_pigpio_interfaces/srv/hal_pigpio_i2c_close.hpp"
#include "hal_pigpio_interfaces/srv/hal_pigpio_i2c_read_byte_data.hpp"
#include "hal_pigpio_interfaces/srv/hal_pigpio_i2c_write_byte_data.hpp"
#include "hal_imu_interfaces/srv/hal_imu_get_handle.hpp"

#define IMU_I2C_BUS 0x1

using I2cOpenFuture_t = rclcpp::Client<hal_pigpio_interfaces::srv::HalPigpioI2cOpen>::SharedFuture;
using I2cCloseFuture_t = rclcpp::Client<hal_pigpio_interfaces::srv::HalPigpioI2cClose>::SharedFuture;

class ImuI2cInit : public rclcpp_lifecycle::LifecycleNode
{
private:
    int32_t imuHandle;
    rclcpp::Service<hal_imu_interfaces::srv::HalImuGetHandle>::SharedPtr getHandleService;
    rclcpp::Client<hal_pigpio_interfaces::srv::HalPigpioI2cOpen>::SharedPtr i2cOpenClient;
    rclcpp::Client<hal_pigpio_interfaces::srv::HalPigpioI2cClose>::SharedPtr i2cCloseClient;

public:
    ImuI2cInit();
    ~ImuI2cInit() = default;
    LifecycleCallbackReturn_t on_configure(const rclcpp_lifecycle::State & previous_state);
    LifecycleCallbackReturn_t on_activate(const rclcpp_lifecycle::State & previous_state);
    LifecycleCallbackReturn_t on_deactivate(const rclcpp_lifecycle::State & previous_state);
    LifecycleCallbackReturn_t on_cleanup(const rclcpp_lifecycle::State & previous_state);
    LifecycleCallbackReturn_t on_shutdown(const rclcpp_lifecycle::State & previous_state);
    LifecycleCallbackReturn_t on_error(const rclcpp_lifecycle::State & previous_state);
    void getHandle(const std::shared_ptr<hal_imu_interfaces::srv::HalImuGetHandle::Request> request,
                   std::shared_ptr<hal_imu_interfaces::srv::HalImuGetHandle::Response> response);
    void initI2cCommunication(void);
};

#endif