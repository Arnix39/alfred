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

#ifndef HAL_PIGPIOIMU_HPP_
#define HAL_PIGPIOIMU_HPP_

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

#endif  // HAL_PIGPIOIMU_HPP_
