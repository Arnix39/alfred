#ifndef HAL_PIGPIO_IMU
#define HAL_PIGPIO_IMU

#include <chrono>
#include <vector>
#define _USE_MATH_DEFINES
#include <cmath>

#include "mpu6050.hpp"

// Services and messages headers (generated)
#include "hal_pigpio_interfaces/srv/hal_pigpio_i2c_imu_reading.hpp"
#include "hal_pigpio_interfaces/msg/hal_pigpio_angles.hpp"

struct Quaternions
{
  float w;
  float x;
  float y;
  float z;
};

struct Angles
{
  float phi;
  float theta;
  float psi;
};

#endif
