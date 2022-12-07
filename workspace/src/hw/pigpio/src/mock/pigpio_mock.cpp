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

#include "pigpiod_if2.h" // NOLINT

#include "mock/raspberrypi_mock.hpp"

static RaspberryPi raspberryPi;

/* I2C */
int i2c_open(int pi, unsigned i2c_bus, unsigned i2c_addr, unsigned i2c_flags)
{
  if (i2c_bus > MAX_I2C_BUS_ID) {
    return PI_BAD_I2C_BUS;
  } else if (i2c_addr != MPU6050_I2C_ADDRESS_HIGH && i2c_addr != MPU6050_I2C_ADDRESS_LOW) {
    return PI_BAD_I2C_ADDR;
  } else {
    return raspberryPi.addI2cHandle(i2c_bus, i2c_addr);
  }
}

int i2c_close(int pi, unsigned handle)
{
  if (!raspberryPi.removeI2cHandle(handle)) {
    return PI_BAD_HANDLE;
  } else {
    return 0;
  }
}

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
  for (uint8_t index = static_cast<uint8_t>(GPIO0);
    index < static_cast<uint8_t>(LAST_GPIO);
    ++index)
  {
    if (index != static_cast<uint8_t>(GPIO0) && index != static_cast<uint8_t>(GPIO1)) {
      raspberryPi.addGpio(static_cast<gpioId>(index));
    }
  }
  return 0;
}

void pigpio_stop(int pi)
{
  raspberryPi.reset();
}

int set_mode(int pi, unsigned gpio, unsigned mode)
{
  return raspberryPi.setGpioType(static_cast<gpioId>(gpio), static_cast<gpioType>(mode));
}

int get_mode(int pi, unsigned gpio)
{
  if (std::get<0>(raspberryPi.getGpioType(static_cast<gpioId>(gpio)))) {
    return std::get<1>(raspberryPi.getGpioType(static_cast<gpioId>(gpio)));
  }
  return -1;
}

int set_pull_up_down(int pi, unsigned gpio, unsigned pud)
{
  return raspberryPi.setGpioResistor(static_cast<gpioId>(gpio), static_cast<gpioResistor>(pud));
}

/* Input */
int callback_cancel(unsigned callback_id) {return 0;}

int gpio_read(int pi, unsigned gpio)
{
  if (std::get<0>(raspberryPi.getGpioLevel(static_cast<gpioId>(gpio)))) {
    return static_cast<int>(std::get<1>(raspberryPi.getGpioLevel(static_cast<gpioId>(gpio))));
  }
  return -3;
}

int callback_ex(int pi, unsigned user_gpio, unsigned edge, CBFuncEx_t f, void * userdata)
{
  gpioCallback callback =
  {false, static_cast<gpioId>(user_gpio), static_cast<gpioEdgeChangeType>(edge)};

  if (std::get<1>(raspberryPi.getGpioType(static_cast<gpioId>(user_gpio))) == gpioType::input) {
    return raspberryPi.setGpioCallback(static_cast<gpioId>(user_gpio), callback);
  }
  return -1;
}

/* Output */
int set_PWM_dutycycle(int pi, unsigned user_gpio, unsigned dutycycle)
{
  gpioPwm pwm = {true, static_cast<uint16_t>(dutycycle), 0};
  return raspberryPi.setGpioPwm(static_cast<gpioId>(user_gpio), pwm);
}

int set_PWM_frequency(int pi, unsigned user_gpio, unsigned frequency)
{
  gpioPwm pwm = {true, 0, static_cast<uint16_t>(frequency)};
  return raspberryPi.setGpioPwm(static_cast<gpioId>(user_gpio), pwm);
}

int gpio_write(int pi, unsigned gpio, unsigned level)
{
  return raspberryPi.setGpioLevel(static_cast<gpioId>(gpio), static_cast<gpioLevel>(level));
}

int gpio_trigger(int pi, unsigned user_gpio, unsigned pulseLen, unsigned level)
{
  return raspberryPi.setGpioLevel(static_cast<gpioId>(user_gpio), static_cast<gpioLevel>(level));
}
