#ifndef HAL_PIGPIO_INPUT
#define HAL_PIGPIO_INPUT

// Services and messages headers (generated)
#include "hal_pigpio_interfaces/srv/hal_pigpio_read_gpio.hpp"
#include "hal_pigpio_interfaces/srv/hal_pigpio_set_callback.hpp"
#include "hal_pigpio_interfaces/srv/hal_pigpio_set_encoder_callback.hpp"
#include "hal_pigpio_interfaces/srv/hal_pigpio_set_motor_direction.hpp"
#include "hal_pigpio_interfaces/msg/hal_pigpio_edge_change.hpp"
#include "hal_pigpio_interfaces/msg/hal_pigpio_encoder_count.hpp"

struct Motor
{
    uint8_t id;
    std::vector<unsigned> gpios;
    int32_t encoderCount;
    bool isDirectionForward;
};

#endif