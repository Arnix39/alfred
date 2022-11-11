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
#include <tuple>

#include "gpioDefinitions.hpp"

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
  neitherEdge,
  risingEdge,
  fallingEdge,
  eitherEdge
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

class RaspberryPi
{
private:
  std::map<uint8_t, gpio> gpios;

public:
  RaspberryPi();
  ~RaspberryPi();
  void addGpio(gpioId gpioId);
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
