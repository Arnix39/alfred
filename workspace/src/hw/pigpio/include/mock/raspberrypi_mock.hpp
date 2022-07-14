#ifndef RASPBERRY_PI_MOCK
#define RASPBERRY_PI_MOCK

#include <cstdint>

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
    uint8_t id;
    gpioType type;
    gpioResistor resistorConfiguration;
    gpioLevel level;
    gpioPwm pwm;
    gpioCallback callback; 
};

class RaspberryPi
{
private:

public:
};

#endif