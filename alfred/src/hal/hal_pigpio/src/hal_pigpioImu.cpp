// Copyright (c) 2022 Arnix Robotix
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "hal_pigpio.hpp"

void Pigpio::resetFifo()
{
  int16_t valueRead;

  valueRead = i2c_read_byte_data(pigpioHandle, i2cHandle, MPU6050_USER_CONTROL_REGISTER);
  if (valueRead < 0) {
    RCLCPP_ERROR(get_logger(), "Failed to reset FIFO!");
    // NOLINTNEXTLINE (impossible to have the curly brace on this line)
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

void Pigpio::readQuaternionData(void)
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
        computeQuaternion(fifoData);
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

void Pigpio::computeQuaternion(char (& data)[MPU6050_DMP_FIFO_QUAT_SIZE])
{
  quaternions_.w =
    static_cast<double>((static_cast<int32_t>(data[0]) <<
    24) |
    (static_cast<int32_t>(data[1]) <<
    16) | (static_cast<int32_t>(data[2]) << 8) | data[3]) / MPU6050_QUATERNION_SCALE;
  quaternions_.x =
    static_cast<double>((static_cast<int32_t>(data[4]) <<
    24) |
    (static_cast<int32_t>(data[5]) <<
    16) | (static_cast<int32_t>(data[6]) << 8) | data[7]) / MPU6050_QUATERNION_SCALE;
  quaternions_.y =
    static_cast<double>((static_cast<int32_t>(data[8]) <<
    24) |
    (static_cast<int32_t>(data[9]) <<
    16) | (static_cast<int32_t>(data[10]) << 8) | data[11]) / MPU6050_QUATERNION_SCALE;
  quaternions_.z =
    static_cast<double>((static_cast<int32_t>(data[12]) <<
    24) |
    (static_cast<int32_t>(data[13]) <<
    16) | (static_cast<int32_t>(data[14]) << 8) | data[15]) / MPU6050_QUATERNION_SCALE;
}

void Pigpio::readAccelerometerData()
{
  char accelerometerData[MPU6050_DATA_SIZE];

  if (i2c_read_i2c_block_data(
      pigpioHandle, i2cHandle, MPU6050_ACCELEROMETER_X_OUT_MSB_REGISTER, accelerometerData,
      MPU6050_DATA_SIZE) == MPU6050_DATA_SIZE)
  {
    linearAcceleration_ = computeImuData(accelerometerData);
  } else {
    RCLCPP_ERROR(get_logger(), "Failed to read accelerometer data!");
    return;
  }
}

void Pigpio::readGyroscopeData()
{
  char gyroscopeData[MPU6050_DATA_SIZE];

  if (i2c_read_i2c_block_data(
      pigpioHandle, i2cHandle, MPU6050_GYROSCOPE_X_OUT_MSB_REGISTER, gyroscopeData,
      MPU6050_DATA_SIZE) == MPU6050_DATA_SIZE)
  {
    angularVelocity_ = computeImuData(gyroscopeData);
  } else {
    RCLCPP_ERROR(get_logger(), "Failed to read gyroscope data!");
    return;
  }
}

Vector3 Pigpio::computeImuData(char (& data)[MPU6050_DATA_SIZE])
{
  Vector3 values;

  /* TODO(Arnix) review computation of raw data */

  values.x = static_cast<uint16_t>((data[0] << 8) | data[1]);
  values.y = static_cast<uint16_t>((data[2] << 8) | data[3]);
  values.z = static_cast<uint16_t>((data[4] << 8) | data[5]);

  return values;
}

void Pigpio::publishImuMessage()
{
  auto message = HalPigpioImuMsg_t();
  auto quaternions = QuaternionMsg_t();
  auto angularVelocity = Vector3Msg_t();
  auto linearAcceleration = Vector3Msg_t();
  auto header = HeaderMsg_t();

  header.frame_id = "Body";
  header.stamp = rclcpp::Clock().now();
  message.header = header;

  std::array<double, 9> covariance_zero = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  quaternions.x = quaternions_.x;
  quaternions.y = quaternions_.y;
  quaternions.z = quaternions_.z;
  quaternions.z = quaternions_.z;

  message.orientation = quaternions;
  message.orientation_covariance = covariance_zero;

  angularVelocity.x = angularVelocity_.x;
  angularVelocity.y = angularVelocity_.y;
  angularVelocity.z = angularVelocity_.z;

  message.angular_velocity = angularVelocity;
  message.angular_velocity_covariance = covariance_zero;

  linearAcceleration.x = linearAcceleration_.x;
  linearAcceleration.y = linearAcceleration_.y;
  linearAcceleration.z = linearAcceleration_.z;

  message.linear_acceleration = linearAcceleration;
  message.linear_acceleration_covariance = covariance_zero;

  imuPublisher->publish(message);
}

void Pigpio::readImuDataAndPublishMessage()
{
  if (isImuReady) {
    readQuaternionData();
    readAccelerometerData();
    readGyroscopeData();
    publishImuMessage();
  }
}

void Pigpio::i2cImuReading(
  const std::shared_ptr<HalPigpioI2cImuReading_t::Request> request,
  std::shared_ptr<HalPigpioI2cImuReading_t::Response> response)
{
  (void)response;

  isImuReady = request->is_imu_ready;
  i2cHandle = request->imu_handle;

  if (isImuReady) {
    resetFifo();
  }
}
