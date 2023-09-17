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

#ifndef HAL_PIGPIOI2C_HPP_
#define HAL_PIGPIOI2C_HPP_

// Services and messages headers (generated)
#include "hal_pigpio_interfaces/srv/hal_pigpio_i2c_open.hpp"
#include "hal_pigpio_interfaces/srv/hal_pigpio_i2c_close.hpp"
#include "hal_pigpio_interfaces/srv/hal_pigpio_i2c_read_byte_data.hpp"
#include "hal_pigpio_interfaces/srv/hal_pigpio_i2c_read_word_data.hpp"
#include "hal_pigpio_interfaces/srv/hal_pigpio_i2c_read_block_data.hpp"
#include "hal_pigpio_interfaces/srv/hal_pigpio_i2c_write_byte_data.hpp"
#include "hal_pigpio_interfaces/srv/hal_pigpio_i2c_write_word_data.hpp"
#include "hal_pigpio_interfaces/srv/hal_pigpio_i2c_write_block_data.hpp"

namespace hal
{
namespace pigpio
{

#define I2C_BUFFER_MAX_BYTES 32

}  // namespace pigpio
}  // namespace hal

#endif  // HAL_PIGPIOI2C_HPP_
