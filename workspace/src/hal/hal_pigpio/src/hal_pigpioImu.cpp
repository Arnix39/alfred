#include "hal_pigpio.hpp"

void Pigpio::resetFifo()
{
  int16_t valueRead;

  valueRead = i2c_read_byte_data(pigpioHandle, i2cHandle, MPU6050_USER_CONTROL_REGISTER);
  if (valueRead < 0) {
    RCLCPP_ERROR(get_logger(), "Failed to reset FIFO!");
  } else if (i2c_write_byte_data(
      pigpioHandle, i2cHandle, MPU6050_USER_CONTROL_REGISTER,
      (static_cast<uint8_t>(valueRead) | (1 << MPU6050_FIFO_RESET_BIT))) != 0)
  {
    RCLCPP_ERROR(get_logger(), "Failed to reset FIFO!");
  }
}

uint16_t Pigpio::readFifoCount()
{
  int16_t valueRead;
  uint16_t fifoCount;

  valueRead = i2c_read_byte_data(pigpioHandle, i2cHandle, MPU6050_FIFO_COUNT_H_REGISTER);
  if (valueRead < 0) {
    RCLCPP_ERROR(get_logger(), "Failed to read the number of bytes in the FIFO!");
    return 0;
  } else {
    fifoCount = static_cast<uint16_t>(valueRead) << 8;
  }

  valueRead = i2c_read_byte_data(pigpioHandle, i2cHandle, MPU6050_FIFO_COUNT_L_REGISTER);
  if (valueRead < 0) {
    RCLCPP_ERROR(get_logger(), "Failed to read the number of bytes in the FIFO!");
    return 0;
  } else {
    fifoCount += static_cast<uint16_t>(valueRead);
  }

  return fifoCount;
}

bool Pigpio::isFifoOverflowed(void)
{
  int16_t interruptStatus;

  interruptStatus = i2c_read_byte_data(pigpioHandle, i2cHandle, MPU6050_INTERRUPT_STATUS_REGISTER);
  if (interruptStatus < 0) {
    RCLCPP_ERROR(get_logger(), "Failed to read interrupt status!");
  } else if (static_cast<uint8_t>(interruptStatus) & MPU6050_FIFO_OVERFLOW) {
    return true;
  }

  return false;
}

void Pigpio::readQuaternions(void)
{
  uint16_t fifoCount = 0;
  char fifoData[MPU6050_DMP_FIFO_QUAT_SIZE];

  if (isFifoOverflowed()) {
    RCLCPP_ERROR(get_logger(), "FIFO has overflowed!");
    resetFifo();
  } else {
    fifoCount = readFifoCount();

    if (fifoCount >= MPU6050_DMP_FIFO_QUAT_SIZE) {
      if (i2c_read_i2c_block_data(
          pigpioHandle, i2cHandle, MPU6050_FIFO_REGISTER, fifoData,
          MPU6050_DMP_FIFO_QUAT_SIZE) == MPU6050_DMP_FIFO_QUAT_SIZE)
      {
        computeQuaternions(fifoData);
      } else {
        RCLCPP_ERROR(get_logger(), "Failed to read FIFO!");
        resetFifo();
        return;
      }

      if (fifoCount > MPU6050_MAX_QUATERNIONS_SAMPLES) {
        resetFifo();
      }
    } else {
      RCLCPP_INFO(get_logger(), "Not enough samples in FIFO.");
    }
  }
}

void Pigpio::computeQuaternions(char (& data)[MPU6050_DMP_FIFO_QUAT_SIZE])
{
  quaternions.w =
    static_cast<float>((static_cast<int32_t>(data[0]) <<
    24) |
    (static_cast<int32_t>(data[1]) <<
    16) | (static_cast<int32_t>(data[2]) << 8) | data[3]) / MPU6050_QUATERNION_SCALE;
  quaternions.x =
    static_cast<float>((static_cast<int32_t>(data[4]) <<
    24) |
    (static_cast<int32_t>(data[5]) <<
    16) | (static_cast<int32_t>(data[6]) << 8) | data[7]) / MPU6050_QUATERNION_SCALE;
  quaternions.y =
    static_cast<float>((static_cast<int32_t>(data[8]) <<
    24) |
    (static_cast<int32_t>(data[9]) <<
    16) | (static_cast<int32_t>(data[10]) << 8) | data[11]) / MPU6050_QUATERNION_SCALE;
  quaternions.z =
    static_cast<float>((static_cast<int32_t>(data[12]) <<
    24) |
    (static_cast<int32_t>(data[13]) <<
    16) | (static_cast<int32_t>(data[14]) << 8) | data[15]) / MPU6050_QUATERNION_SCALE;
}

void Pigpio::publishAngles(void)
{
  auto message = hal_pigpio_interfaces::msg::HalPigpioAngles();

  message.phi = angles.phi;
  message.theta = angles.theta;
  message.psi = angles.psi;
  anglesPublisher->publish(message);
}

void Pigpio::computeAngles()
{
  // phi (x-axis rotation)
  float tanPhi = 2 * (quaternions.y * quaternions.z - quaternions.w * quaternions.x);
  float quadrantPhi = 2 * (quaternions.w * quaternions.w + quaternions.z * quaternions.z) - 1;
  angles.phi = std::atan2(tanPhi, quadrantPhi) * 180 / M_PI;

  // theta (y-axis rotation)
  float sinTheta = 2 * (quaternions.x * quaternions.z + quaternions.w * quaternions.y);
  angles.theta = -std::asin(sinTheta) * 180 / M_PI;

  // psi (z-axis rotation)
  float tanPsi = 2 * (quaternions.x * quaternions.y - quaternions.w * quaternions.z);
  float quadrantPsi = 2 * (quaternions.w * quaternions.w + quaternions.x * quaternions.x) - 1;
  angles.psi = std::atan2(tanPsi, quadrantPsi) * 180 / M_PI;
}

void Pigpio::readQuaternionsAndPublishAngles()
{
  if (isImuReady) {
    readQuaternions();
    computeAngles();
    publishAngles();
  }
}

void Pigpio::i2cImuReading(
  const std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioI2cImuReading::Request> request,
  std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioI2cImuReading::Response> response)
{
  (void)response;

  isImuReady = request->is_imu_ready;
  i2cHandle = request->imu_handle;

  if (isImuReady) {
    resetFifo();
  }
}
