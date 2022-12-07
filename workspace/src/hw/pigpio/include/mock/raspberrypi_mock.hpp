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

#ifndef MOCK__RASPBERRYPI_MOCK_HPP_
#define MOCK__RASPBERRYPI_MOCK_HPP_

#include <cstdint>
#include <map>
#include <utility>
#include <vector>
#include <tuple>

#include "gpioDefinitions.hpp"

#define MPU6050_I2C_ADDRESS_LOW 0x67
#define MPU6050_I2C_ADDRESS_HIGH 0x68
#define MAX_I2C_BUS_ID 1

enum gpioType
{
  input,
  output
};

enum gpioResistor
{
  off,
  pullDown,
  pullUp
};

enum gpioLevel
{
  low,
  high
};

struct gpioPwm
{
  bool isEnabled;
  uint16_t dutycycle;
  uint16_t frequency;
};

enum gpioEdgeChangeType
{
  risingEdge,
  fallingEdge,
  eitherEdge,
  neitherEdge
};

struct gpioCallback
{
  bool isInitialised;
  uint16_t id;
  gpioEdgeChangeType edgeChangeType;
};

struct gpio
{
  gpioType type;
  gpioResistor resistorConfiguration;
  gpioLevel level;
  gpioPwm pwm;
  gpioCallback callback;
};

struct i2cHandle
{
  uint32_t handle;
  uint8_t busAddress;
  uint8_t deviceAddress;
};

class RaspberryPi
{
private:
  std::map<uint8_t, gpio> gpios;
  std::vector<i2cHandle> i2cHandles;

public:
  RaspberryPi();
  ~RaspberryPi();
  void reset(void);
  void addGpio(gpioId gpioId);
  int32_t addI2cHandle(uint8_t busAddress, uint8_t deviceAddress);
  bool removeI2cHandle(uint32_t handle);
  int setGpioType(gpioId gpioId, gpioType type);
  int setGpioResistor(gpioId gpioId, gpioResistor resistorConfiguration);
  int setGpioLevel(gpioId gpioId, gpioLevel level);
  int setGpioPwm(gpioId gpioId, gpioPwm pwm);
  int setGpioCallback(gpioId gpioId, gpioCallback callback);
  std::tuple<bool, gpioType> getGpioType(gpioId gpioId);
  std::tuple<bool, gpioResistor> getGpioResistor(gpioId gpioId);
  std::tuple<bool, gpioLevel> getGpioLevel(gpioId gpioId);
  std::tuple<bool, gpioPwm> getGpioPwm(gpioId gpioId);
  std::tuple<bool, gpioCallback> getGpioCallback(gpioId gpioId);
};

#endif  // MOCK__RASPBERRYPI_MOCK_HPP_
