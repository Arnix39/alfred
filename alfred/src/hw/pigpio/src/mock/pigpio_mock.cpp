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

static hw::raspberry_pi::RaspberryPi raspberryPi;

/* I2C */
int i2c_open(int pi, unsigned i2c_bus, unsigned i2c_addr, unsigned i2c_flags)
{
  if (pi < 0) {
    return PI_NO_HANDLE;
  } else if (i2c_bus > MAX_I2C_BUS_ID) {
    return PI_BAD_I2C_BUS;
  } else if (i2c_addr != MPU6050_I2C_ADDRESS_HIGH && i2c_addr != MPU6050_I2C_ADDRESS_LOW) {
    return PI_BAD_I2C_ADDR;
  } else if (raspberryPi.i2cHandleExists(i2c_bus, i2c_addr)) {
    return PI_I2C_OPEN_FAILED;
  } else {
    return static_cast<int>(raspberryPi.addI2cHandle(i2c_bus, i2c_addr));
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

int i2c_read_byte_data(int pi, unsigned handle, unsigned i2c_reg)
{
  if (raspberryPi.i2cHandleExists(handle) && raspberryPi.registerExists(i2c_reg)) {
    int16_t result = raspberryPi.readRegister(handle, i2c_reg, 0);
    if (result >= 0) {
      return static_cast<uint8_t>(result);
    } else {
      return PI_I2C_READ_FAILED;
    }
  } else if (!raspberryPi.i2cHandleExists(handle)) {
    return PI_BAD_HANDLE;
  } else {
    return PI_BAD_PARAM;
  }
}

int i2c_read_word_data(int pi, unsigned handle, unsigned i2c_reg)
{
  uint8_t bytes[2] = {0};
  if (raspberryPi.i2cHandleExists(handle) && raspberryPi.registerExists(i2c_reg)) {
    for (int index = 0; index < 2; ++index) {
      int16_t result = raspberryPi.readRegister(handle, i2c_reg, index);
      if (result < 0) {
        return PI_I2C_READ_FAILED;
      } else {
        bytes[index] = static_cast<uint8_t>(result);
      }
    }
    return static_cast<int>(bytes[0] << 8 | bytes[1]);
  } else if (!raspberryPi.i2cHandleExists(handle)) {
    return PI_BAD_HANDLE;
  } else {
    return PI_BAD_PARAM;
  }
}

int i2c_read_i2c_block_data(int pi, unsigned handle, unsigned i2c_reg, char * buf, unsigned count)
{
  if (raspberryPi.i2cHandleExists(handle) && raspberryPi.registerExists(i2c_reg)) {
    for (int index = 0; index < count; ++index) {
      int16_t result = raspberryPi.readRegister(handle, i2c_reg, index);
      if (result < 0) {
        return PI_I2C_READ_FAILED;
      } else {
        buf[index] = static_cast<uint8_t>(result);
      }
    }
    return count;
  } else if (!raspberryPi.i2cHandleExists(handle)) {
    return PI_BAD_HANDLE;
  } else {
    return PI_BAD_PARAM;
  }
}

int i2c_write_byte_data(int pi, unsigned handle, unsigned i2c_reg, unsigned bVal)
{
  std::vector<uint8_t> bytes = {static_cast<uint8_t>(bVal)};

  if (raspberryPi.i2cHandleExists(handle) && raspberryPi.registerExists(i2c_reg)) {
    if (raspberryPi.writeRegister(handle, i2c_reg, bytes)) {
      return 0;
    } else {
      return PI_I2C_WRITE_FAILED;
    }
  } else if (!raspberryPi.i2cHandleExists(handle)) {
    return PI_BAD_HANDLE;
  } else {
    return PI_BAD_PARAM;
  }
}

int i2c_write_word_data(int pi, unsigned handle, unsigned i2c_reg, unsigned wVal)
{
  std::vector<uint8_t> bytes = {static_cast<uint8_t>(wVal >> 8), static_cast<uint8_t>(wVal & 0xFF)};

  if (raspberryPi.i2cHandleExists(handle) && raspberryPi.registerExists(i2c_reg)) {
    if (!raspberryPi.writeRegister(handle, i2c_reg, bytes)) {
      return PI_I2C_WRITE_FAILED;
    }

    return 0;
  } else if (!raspberryPi.i2cHandleExists(handle)) {
    return PI_BAD_HANDLE;
  } else {
    return PI_BAD_PARAM;
  }
}

int i2c_write_i2c_block_data(int pi, unsigned handle, unsigned i2c_reg, char * buf, unsigned count)
{
  std::vector<uint8_t> bytes;
  if (raspberryPi.i2cHandleExists(handle) && raspberryPi.registerExists(i2c_reg)) {
    for (int index = 0; index < count; ++index) {
      bytes.push_back(buf[index]);
    }
    if (!raspberryPi.writeRegister(handle, i2c_reg, bytes)) {
      return PI_I2C_WRITE_FAILED;
    }
    return 0;
  } else if (!raspberryPi.i2cHandleExists(handle)) {
    return PI_BAD_HANDLE;
  } else {
    return PI_BAD_PARAM;
  }
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
  if (static_cast<hw::raspberry_pi::gpioType>(mode) != hw::raspberry_pi::gpioType::input &&
    static_cast<hw::raspberry_pi::gpioType>(mode) != hw::raspberry_pi::gpioType::output)
  {
    return PI_BAD_MODE;
  } else if (std::get<0>(raspberryPi.getGpioType(static_cast<gpioId>(gpio)))) {
    return raspberryPi.setGpioType(
      static_cast<gpioId>(gpio),
      static_cast<hw::raspberry_pi::gpioType>(mode));
  } else {
    return PI_BAD_GPIO;
  }
}

int get_mode(int pi, unsigned gpio)
{
  if (std::get<0>(raspberryPi.getGpioType(static_cast<gpioId>(gpio)))) {
    return std::get<1>(raspberryPi.getGpioType(static_cast<gpioId>(gpio)));
  } else {
    return PI_BAD_GPIO;
  }
}

int set_pull_up_down(int pi, unsigned gpio, unsigned pud)
{
  if (static_cast<hw::raspberry_pi::gpioResistor>(pud) != hw::raspberry_pi::gpioResistor::off &&
    static_cast<hw::raspberry_pi::gpioResistor>(pud) != hw::raspberry_pi::gpioResistor::pullDown &&
    static_cast<hw::raspberry_pi::gpioResistor>(pud) != hw::raspberry_pi::gpioResistor::pullUp)
  {
    return PI_BAD_PUD;
  } else if (std::get<0>(raspberryPi.getGpioType(static_cast<gpioId>(gpio)))) {
    return raspberryPi.setGpioResistor(
      static_cast<gpioId>(gpio),
      static_cast<hw::raspberry_pi::gpioResistor>(pud));
  } else {
    return PI_BAD_GPIO;
  }
}

/* Input */
int callback_cancel(unsigned callback_id) {return 0;}

int gpio_read(int pi, unsigned gpio)
{
  if (std::get<0>(raspberryPi.getGpioLevel(static_cast<gpioId>(gpio)))) {
    return static_cast<int>(std::get<1>(raspberryPi.getGpioLevel(static_cast<gpioId>(gpio))));
  }
  return PI_BAD_GPIO;
}

int callback(int pi, unsigned user_gpio, unsigned edge, CBFunc_t f)
{
  hw::raspberry_pi::gpioCallback callback =
  {false, static_cast<gpioId>(user_gpio), static_cast<hw::raspberry_pi::gpioEdgeChangeType>(edge)};

  if (std::get<1>(raspberryPi.getGpioType(static_cast<gpioId>(user_gpio))) ==
    hw::raspberry_pi::gpioType::input)
  {
    return raspberryPi.setGpioCallback(static_cast<gpioId>(user_gpio), callback);
  }
  return pigif_bad_malloc;
}

int callback_ex(int pi, unsigned user_gpio, unsigned edge, CBFuncEx_t f, void * userdata)
{
  hw::raspberry_pi::gpioCallback callback =
  {false, static_cast<gpioId>(user_gpio), static_cast<hw::raspberry_pi::gpioEdgeChangeType>(edge)};

  if (std::get<1>(raspberryPi.getGpioType(static_cast<gpioId>(user_gpio))) ==
    hw::raspberry_pi::gpioType::input)
  {
    return raspberryPi.setGpioCallback(static_cast<gpioId>(user_gpio), callback);
  }
  return pigif_bad_malloc;
}

/* Output */
int set_PWM_dutycycle(int pi, unsigned user_gpio, unsigned dutycycle)
{
  hw::raspberry_pi::gpioPwm pwm;
  if (std::get<0>(raspberryPi.getGpioPwm(static_cast<gpioId>(user_gpio)))) {
    if (std::get<1>(raspberryPi.getGpioPwm(static_cast<gpioId>(user_gpio))).isEnabled) {
      pwm = std::get<1>(raspberryPi.getGpioPwm(static_cast<gpioId>(user_gpio)));
      pwm.dutycycle = dutycycle;
      return raspberryPi.setGpioPwm(static_cast<gpioId>(user_gpio), pwm);
    } else {
      pwm = {true, static_cast<uint8_t>(dutycycle), 0};
      return raspberryPi.setGpioPwm(static_cast<gpioId>(user_gpio), pwm);
    }
  } else {
    return PI_BAD_USER_GPIO;
  }
}

int get_PWM_dutycycle(int pi, unsigned user_gpio)
{
  if (std::get<0>(raspberryPi.getGpioPwm(static_cast<gpioId>(user_gpio)))) {
    if (std::get<1>(raspberryPi.getGpioPwm(static_cast<gpioId>(user_gpio))).isEnabled) {
      return std::get<1>(raspberryPi.getGpioPwm(static_cast<gpioId>(user_gpio))).dutycycle;
    } else {
      return PI_NOT_PWM_GPIO;
    }
  } else {
    return PI_BAD_USER_GPIO;
  }
}

int set_PWM_frequency(int pi, unsigned user_gpio, unsigned frequency)
{
  hw::raspberry_pi::gpioPwm pwm;
  if (std::get<0>(raspberryPi.getGpioPwm(static_cast<gpioId>(user_gpio)))) {
    if (std::get<1>(raspberryPi.getGpioPwm(static_cast<gpioId>(user_gpio))).isEnabled) {
      pwm = std::get<1>(raspberryPi.getGpioPwm(static_cast<gpioId>(user_gpio)));
      pwm.frequency = frequency;
      return raspberryPi.setGpioPwm(static_cast<gpioId>(user_gpio), pwm);
    } else {
      pwm = {true, 0, static_cast<uint16_t>(frequency)};
      return raspberryPi.setGpioPwm(static_cast<gpioId>(user_gpio), pwm);
    }
  } else {
    return PI_BAD_USER_GPIO;
  }
}

int get_PWM_frequency(int pi, unsigned user_gpio)
{
  if (std::get<0>(raspberryPi.getGpioPwm(static_cast<gpioId>(user_gpio)))) {
    if (std::get<1>(raspberryPi.getGpioPwm(static_cast<gpioId>(user_gpio))).isEnabled) {
      return std::get<1>(raspberryPi.getGpioPwm(static_cast<gpioId>(user_gpio))).frequency;
    } else {
      return PI_NOT_PWM_GPIO;
    }
  } else {
    return PI_BAD_USER_GPIO;
  }
}

int gpio_write(int pi, unsigned gpio, unsigned level)
{
  auto result =
    raspberryPi.setGpioLevel(
    static_cast<gpioId>(gpio),
    static_cast<hw::raspberry_pi::gpioLevel>(level));
  if (result == 0) {
    return 0;
  } else if (result == 1) {
    return PI_NOT_PERMITTED;
  } else {
    return PI_BAD_GPIO;
  }
}

int gpio_trigger(int pi, unsigned user_gpio, unsigned pulseLen, unsigned level)
{
  auto result =
    raspberryPi.setGpioLevel(
    static_cast<gpioId>(user_gpio),
    static_cast<hw::raspberry_pi::gpioLevel>(level));
  if (result == 0) {
    return 0;
  } else if (result == 1) {
    return PI_NOT_PERMITTED;
  } else {
    return PI_BAD_GPIO;
  }
}
