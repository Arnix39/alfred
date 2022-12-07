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

#include "mock/raspberrypi_mock.hpp"

RaspberryPi::RaspberryPi()
: gpios({}),
  i2cHandles({})
{
}

RaspberryPi::~RaspberryPi()
{
  gpios = {};
  i2cHandles = {};
}

void RaspberryPi::reset(void)
{
  gpios = {};
  i2cHandles = {};
}

void RaspberryPi::addGpio(gpioId gpioId)
{
  gpioPwm gpioPwm({false, 0, 0});
  gpioCallback gpioCallback({false, 0, neitherEdge});
  gpio newGpio({gpioType::input, gpioResistor::off, gpioLevel::low, gpioPwm, gpioCallback});

  gpios.insert({gpioId, newGpio});
}

int32_t RaspberryPi::addI2cHandle(uint8_t busAddress, uint8_t deviceAddress)
{
  for (int index = 0; index < i2cHandles.size(); ++index) {
    if ((i2cHandles.at(index).at(1) == busAddress) &&
      (i2cHandles.at(index).at(2) == deviceAddress))
    {
      return index;
    }
  }

  i2cHandles.push_back({static_cast<uint8_t>(i2cHandles.size()), busAddress, deviceAddress});
  return i2cHandles.size() - 1;
}

bool RaspberryPi::removeI2cHandle(uint32_t handle)
{
  for (int index = 0; index < i2cHandles.size(); ++index) {
    if (i2cHandles.at(index).at(0) == handle) {
      i2cHandles.erase(i2cHandles.begin() + index);
      return true;
    }
  }

  return false;
}

int RaspberryPi::setGpioType(gpioId gpioId, gpioType type)
{
  auto gpio = gpios.find(gpioId);
  if (gpio != gpios.end()) {
    gpio->second.type = type;
    return 0;
  }
  return -1;
}

int RaspberryPi::setGpioResistor(gpioId gpioId, gpioResistor resistorConfiguration)
{
  auto gpio = gpios.find(gpioId);
  if (gpio != gpios.end()) {
    gpio->second.resistorConfiguration = resistorConfiguration;
    return 0;
  }
  return -1;
}

int RaspberryPi::setGpioLevel(gpioId gpioId, gpioLevel level)
{
  auto gpio = gpios.find(gpioId);
  if (gpio != gpios.end()) {
    gpio->second.level = level;

    if (std::get<1>(getGpioType(gpioId)) == output) {
      return 0;
    } else {
      return 1;
    }
  }
  return -1;
}

int RaspberryPi::setGpioPwm(gpioId gpioId, gpioPwm pwm)
{
  auto gpio = gpios.find(gpioId);
  if (gpio != gpios.end() && std::get<1>(getGpioType(gpioId)) == output) {
    gpio->second.pwm = pwm;
    return 0;
  }
  return -2;
}

int RaspberryPi::setGpioCallback(gpioId gpioId, gpioCallback callback)
{
  auto gpio = gpios.find(gpioId);
  if (gpio != gpios.end()) {
    gpio->second.callback = callback;
    return gpio->second.callback.id;
  }
  return -1;
}

std::tuple<bool, gpioType> RaspberryPi::getGpioType(gpioId gpioId)
{
  auto gpio = gpios.find(gpioId);
  if (gpio != gpios.end()) {
    return std::make_tuple(true, gpio->second.type);
  } else {
    return std::make_tuple(false, gpioType::input);
  }
}

std::tuple<bool, gpioResistor> RaspberryPi::getGpioResistor(gpioId gpioId)
{
  auto gpio = gpios.find(gpioId);
  if (gpio != gpios.end()) {
    return std::make_tuple(true, gpio->second.resistorConfiguration);
  } else {
    return std::make_tuple(false, gpioResistor::off);
  }
}

std::tuple<bool, gpioLevel> RaspberryPi::getGpioLevel(gpioId gpioId)
{
  auto gpio = gpios.find(gpioId);
  if (gpio != gpios.end()) {
    return std::make_tuple(true, gpio->second.level);
  } else {
    return std::make_tuple(false, gpioLevel::low);
  }
}

std::tuple<bool, gpioPwm> RaspberryPi::getGpioPwm(gpioId gpioId)
{
  gpioPwm gpioPwm({false, 0, 0});

  auto gpio = gpios.find(gpioId);
  if (gpio != gpios.end()) {
    return std::make_tuple(true, gpio->second.pwm);
  } else {
    return std::make_tuple(false, gpioPwm);
  }
}

std::tuple<bool, gpioCallback> RaspberryPi::getGpioCallback(gpioId gpioId)
{
  gpioCallback gpioCallback({false, 0, neitherEdge});

  auto gpio = gpios.find(gpioId);
  if (gpio != gpios.end()) {
    return std::make_tuple(true, gpio->second.callback);
  } else {
    return std::make_tuple(false, gpioCallback);
  }
}
