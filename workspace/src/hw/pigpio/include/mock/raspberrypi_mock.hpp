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
  ~RaspberryPi() = default;
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
