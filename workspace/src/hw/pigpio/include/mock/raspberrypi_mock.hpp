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

#ifndef RASPBERRY_PI_MOCK
#define RASPBERRY_PI_MOCK

#include <cstdint>
#include <map>
#include <tuple>

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
  void addGpio(uint8_t gpioId);
  void setGpioType(uint8_t gpioId, gpioType type);
  void setGpioResistor(uint8_t gpioId, gpioResistor resistorConfiguration);
  void setGpioLevel(uint8_t gpioId, gpioLevel level);
  void setGpioPwm(uint8_t gpioId, gpioPwm pwm);
  void setGpioCallback(uint8_t gpioId, gpioCallback callback);
  std::tuple<bool, gpioType> getGpioType(uint8_t gpioId);
  std::tuple<bool, gpioResistor> getGpioResistor(uint8_t gpioId);
  std::tuple<bool, gpioLevel> getGpioLevel(uint8_t gpioId);
  std::tuple<bool, gpioPwm> getGpioPwm(uint8_t gpioId);
  std::tuple<bool, gpioCallback> getGpioCallback(uint8_t gpioId);
};

#endif
