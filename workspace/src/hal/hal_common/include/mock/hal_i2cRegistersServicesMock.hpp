// Copyright (c) 2023 Arnix Robotix
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

#ifndef MOCK__HAL_I2CREGISTERSSERVICESMOCK_HPP_
#define MOCK__HAL_I2CREGISTERSSERVICESMOCK_HPP_

#include "pigpiod_if2.h" // NOLINT

#define IMU_I2C_BUS 0x1
#define I2C_BUFFER_MAX_BYTES 32

void closePigpio(int32_t i2cHandle);

#endif  // MOCK__HAL_I2CREGISTERSSERVICESMOCK_HPP_
