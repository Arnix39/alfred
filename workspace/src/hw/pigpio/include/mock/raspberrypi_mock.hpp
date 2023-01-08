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
#include <algorithm>

#include "gpioDefinitions.hpp"

#define MPU6050_I2C_ADDRESS_LOW 0x67
#define MPU6050_I2C_ADDRESS_HIGH 0x68
#define MPU6050_USER_CONTROL_REGISTER 0x6A
#define MPU6050_FIFO_COUNT_H_REGISTER 0x72
#define MPU6050_FIFO_COUNT_L_REGISTER 0x73
#define MPU6050_FIFO_REGISTER 0x74
#define MPU6050_INTERRUPT_STATUS_REGISTER 0x3A
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

struct i2cHandle_t
{
  uint32_t handle;
  uint8_t bus;
  uint8_t device;
};

struct i2cRegister_t
{
  uint32_t device;
  uint32_t address;
  uint16_t size;
  std::vector<uint8_t> bytes;
};

class RaspberryPi
{
private:
  std::map<uint8_t, gpio> gpios;
  std::vector<i2cHandle_t> i2cHandles;
  std::vector<i2cRegister_t> i2cRegisters;

public:
  RaspberryPi();
  ~RaspberryPi();
  void reset(void);
  void addGpio(gpioId gpioId);
  uint32_t addI2cHandle(uint8_t bus, uint8_t device);
  bool i2cHandleExists(uint8_t bus, uint8_t device);
  bool i2cHandleExists(uint32_t handle);
  bool removeI2cHandle(uint32_t handle);
  bool registerExists(uint32_t address);
  int16_t readRegister(uint32_t i2cHandle, uint32_t address, uint16_t byteIndex);
  bool writeRegister(uint32_t i2cHandle, uint32_t address, std::vector<uint8_t> bytes);
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
