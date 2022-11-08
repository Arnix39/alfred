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

#include "pigpiod_if2.h"

#include "mock/raspberrypi_mock.hpp"

static RaspberryPi raspberryPi;

/* I2C */
int i2c_open(int pi, unsigned i2c_bus, unsigned i2c_addr, unsigned i2c_flags) {return 0;}
int i2c_close(int pi, unsigned handle) {return 0;}
int i2c_read_byte_data(int pi, unsigned handle, unsigned i2c_reg) {return 0;}
int i2c_read_word_data(int pi, unsigned handle, unsigned i2c_reg) {return 0;}
int i2c_read_i2c_block_data(int pi, unsigned handle, unsigned i2c_reg, char * buf, unsigned count)
{
  return 0;
}
int i2c_write_byte_data(int pi, unsigned handle, unsigned i2c_reg, unsigned bVal) {return 0;}
int i2c_write_word_data(int pi, unsigned handle, unsigned i2c_reg, unsigned wVal) {return 0;}
int i2c_write_i2c_block_data(int pi, unsigned handle, unsigned i2c_reg, char * buf, unsigned count)
{
  return 0;
}

/* Init */
int pigpio_start(const char * addrStr, const char * portStr) 
{
  for (uint8_t index = 1; index <= 40; ++index)
  {
    if (index != 27 && index != 28)
    {
      raspberryPi.addGpio(index);
    }
  }
  return 0;
}

void pigpio_stop(int pi) {}

int set_mode(int pi, unsigned gpio, unsigned mode)
{
  return raspberryPi.setGpioType(gpio, static_cast<gpioType>(mode));
}

int get_mode(int pi, unsigned gpio)
{
  if (std::get<0>(raspberryPi.getGpioType(gpio)))
  {
    return std::get<1>(raspberryPi.getGpioType(gpio));
  }
  return -1;
}

int set_pull_up_down(int pi, unsigned gpio, unsigned pud)
{
  return raspberryPi.setGpioResistor(gpio, static_cast<gpioResistor>(pud));
}

/* Input */
int callback_cancel(unsigned callback_id) {return 0;}
int gpio_read(int pi, unsigned gpio) {return 0;}
int callback_ex(int pi, unsigned user_gpio, unsigned edge, CBFuncEx_t f, void * userdata)
{
  return 0;
}

/* Output */
int set_PWM_dutycycle(int pi, unsigned user_gpio, unsigned dutycycle) {return 0;}
int set_PWM_frequency(int pi, unsigned user_gpio, unsigned frequency) {return 0;}
int gpio_write(int pi, unsigned gpio, unsigned level) {return 0;}
int gpio_trigger(int pi, unsigned user_gpio, unsigned pulseLen, unsigned level) {return 0;}
