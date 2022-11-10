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
: gpios({})
{
}

RaspberryPi::~RaspberryPi()
{
  gpios = {};
}

void RaspberryPi::addGpio(gpioId gpioId)
{
  gpioPwm gpioPwm({false, 0, 0});
  gpioCallback gpioCallback({false, 0, neitherEdge});
  gpio newGpio({gpioType::input, gpioResistor::off, gpioLevel::low, gpioPwm, gpioCallback});

  gpios.insert({gpioId, newGpio});
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
  if (gpio != gpios.end() && std::get<1>(getGpioType(gpioId)) == output) {
    gpio->second.level = level;
    return 0;
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
    return 0;
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
