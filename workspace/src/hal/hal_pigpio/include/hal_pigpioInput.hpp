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

#ifndef HAL_PIGPIOINPUT_HPP_
#define HAL_PIGPIOINPUT_HPP_

#include <vector>

// Services and messages headers (generated)
#include "hal_pigpio_interfaces/srv/hal_pigpio_read_gpio.hpp"
#include "hal_pigpio_interfaces/srv/hal_pigpio_set_callback.hpp"
#include "hal_pigpio_interfaces/srv/hal_pigpio_set_encoder_callback.hpp"
#include "hal_pigpio_interfaces/srv/hal_pigpio_set_motor_direction.hpp"
#include "hal_pigpio_interfaces/msg/hal_pigpio_edge_change.hpp"
#include "hal_pigpio_interfaces/msg/hal_pigpio_encoder_count.hpp"

struct Motor
{
  uint8_t id;
  std::vector<unsigned> gpios;
  int32_t encoderCount;
  bool isDirectionForward;
};

#endif  // HAL_PIGPIOINPUT_HPP_
