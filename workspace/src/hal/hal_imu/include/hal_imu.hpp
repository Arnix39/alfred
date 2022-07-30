#ifndef HAL_IMU
#define HAL_IMU

#include "rclcpp_action/rclcpp_action.hpp"

#include "common.hpp"
#include "mpu6050.hpp"
#include "hal_i2cRegistersServices.hpp"

// Services and messages headers (generated)
#include "hal_pigpio_interfaces/srv/hal_pigpio_i2c_read_byte_data.hpp"
#include "hal_pigpio_interfaces/srv/hal_pigpio_i2c_write_byte_data.hpp"
#include "hal_pigpio_interfaces/srv/hal_pigpio_i2c_write_block_data.hpp"
#include "hal_pigpio_interfaces/srv/hal_pigpio_i2c_imu_reading.hpp"
#include "hal_imu_interfaces/msg/hal_imu.hpp"
#include "hal_imu_interfaces/action/hal_imu_write_dmp.hpp"

#define IMU_GYROSCOPE_X_OFFSET 0
#define IMU_GYROSCOPE_Y_OFFSET -34
#define IMU_GYROSCOPE_Z_OFFSET -9

#define IMU_ACCELEROMETER_X_OFFSET -556
#define IMU_ACCELEROMETER_Y_OFFSET -1188
#define IMU_ACCELEROMETER_Z_OFFSET 873

struct SensorBias
{
  const char axis;
  const int16_t bias;
  const uint8_t msbRegister;
  const uint8_t lsbRegister;
};

using i2cImuReadingFuture_t =
  rclcpp::Client<hal_pigpio_interfaces::srv::HalPigpioI2cImuReading>::SharedFuture;

using HalImuWriteDmpAction = hal_imu_interfaces::action::HalImuWriteDmp;
using HalImuWriteDmpGoal = rclcpp_action::ClientGoalHandle<HalImuWriteDmpAction>;

class Imu : public rclcpp_lifecycle::LifecycleNode
{
private:
  int32_t imuHandle;

  rclcpp::Client<hal_pigpio_interfaces::srv::HalPigpioI2cReadByteData>::SharedPtr
    i2cReadByteDataClient;
  rclcpp::Client<hal_pigpio_interfaces::srv::HalPigpioI2cWriteByteData>::SharedPtr
    i2cWriteByteDataClient;
  rclcpp::Client<hal_pigpio_interfaces::srv::HalPigpioI2cWriteBlockData>::SharedPtr
    i2cWriteBlockDataClient;
  rclcpp::Client<hal_pigpio_interfaces::srv::HalPigpioI2cImuReading>::SharedPtr i2cImuReadingClient;
  rclcpp::Client<hal_imu_interfaces::srv::HalImuGetHandle>::SharedPtr imuGetHandleClient;

  rclcpp_action::Client<hal_imu_interfaces::action::HalImuWriteDmp>::SharedPtr imuDmpWritingClient;

  void goal_response_callback(HalImuWriteDmpGoal::SharedPtr goal_handle);
  void feedback_callback(
    HalImuWriteDmpGoal::SharedPtr,
    const std::shared_ptr<const HalImuWriteDmpAction::Feedback> feedback);
  void result_callback(const HalImuWriteDmpGoal::WrappedResult & result);

public:
  Imu();
  ~Imu() = default;

  LifecycleCallbackReturn_t on_configure(const rclcpp_lifecycle::State & previous_state);
  LifecycleCallbackReturn_t on_activate(const rclcpp_lifecycle::State & previous_state);
  LifecycleCallbackReturn_t on_deactivate(const rclcpp_lifecycle::State & previous_state);
  LifecycleCallbackReturn_t on_cleanup(const rclcpp_lifecycle::State & previous_state);
  LifecycleCallbackReturn_t on_shutdown(const rclcpp_lifecycle::State & previous_state);
  LifecycleCallbackReturn_t on_error(const rclcpp_lifecycle::State & previous_state);

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
};

#endif
