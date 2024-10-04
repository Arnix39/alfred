# hal_pigpio_interfaces package

## Overview

This interface package defines:
- The following messages:
    - `HalPigpioEdgeChange` that contains data about one digital input level change (the GPIO number, the type of change and a timestamp).
    - `HalPigpioEncoderCount` that contains all the motors' raw encoder count values.

- The following services:
    - **Initialization**
    - `HalPigpioGetMode` that gets the mode of the given GPIO by calling [`get_mode`](https://abyz.me.uk/rpi/pigpio/pdif2.html#get_mode). Returns `true` and the mode if it succeeded, `false` and `PI_BAD_GPIO` otherwise.
    - `HalPigpioSetInputMode` that sets the mode of the given GPIO to input by calling [`set_mode`](https://abyz.me.uk/rpi/pigpio/pdif2.html#set_mode) with `PI_INPUT`. Returns `true` if it succeeded, `false` otherwise.
    - `HalPigpioSetOutputMode` that sets the mode of the given GPIO to output by calling [`set_mode`](https://abyz.me.uk/rpi/pigpio/pdif2.html#set_mode) with `PI_OUTPUT`. Returns `true` if it succeeded, `false` otherwise.
    - `HalPigpioSetPullDown` that sets the given GPIO pull-down resistor by calling [`set_pull_up_down`](https://abyz.me.uk/rpi/pigpio/pdif2.html#set_pull_up_down) with `PI_PUD_DOWN`. Returns `true` if it succeeded, `false` otherwise.
    - `HalPigpioSetPullUp` that sets the given GPIO pull-up resistor by calling [`set_pull_up_down`](https://abyz.me.uk/rpi/pigpio/pdif2.html#set_pull_up_down) with `PI_PUD_UP`. Returns `true` if it succeeded, `false` otherwise.
    - `HalPigpioClearResistor` that clears the given GPIO pull-up/down resistor by calling [`set_pull_up_down`](https://abyz.me.uk/rpi/pigpio/pdif2.html#set_pull_up_down) with `PI_PUD_OFF`. Returns `true` if it succeeded, `false` otherwise.
    - **Input**
    - `HalPigpioReadGpio` that reads the level of the given GPIO by calling [`gpio_read`](https://abyz.me.uk/rpi/pigpio/pdif2.html#gpio_read). Returns `true` and the level if it succeeded, `false` and `PI_BAD_GPIO` otherwise.
    - `HalPigpioSetCallback` that sets a callback for a given level change (`edge_change_type`) of the given GPIO by calling [`callback_ex`](https://abyz.me.uk/rpi/pigpio/pdif2.html#callback_ex). Returns `true` and the callback ID if it succeeded, `false` and an error code otherwise.
    - `HalPigpioSetEncoderCallback` that sets a callback for a given level change (`edge_change_type`) of the given GPIO associated with a motor's encoder channel by calling [`callback_ex`](https://abyz.me.uk/rpi/pigpio/pdif2.html#callback_ex). Returns `true` and the callback ID if it succeeded, `false` and an error code otherwise.
    - **Output**
    - `HalPigpioSetGpioLow` that sets the level on the given GPIO to low by calling [`gpio_write`](https://abyz.me.uk/rpi/pigpio/pdif2.html#gpio_write) with `PI_LOW`. Returns `true` if it succeeded, `false` otherwise.
    - `HalPigpioSetGpioHigh` that sets the level on the given GPIO to high by calling [`gpio_write`](https://abyz.me.uk/rpi/pigpio/pdif2.html#gpio_write) with `PI_HIGH`. Returns `true` if it succeeded, `false` otherwise.
    - `HalPigpioSetPwmFrequency` that sets the frequency of the PWM for the given GPIO to `frequency` by calling [`set_PWM_frequency`](https://abyz.me.uk/rpi/pigpio/pdif2.html#set_PWM_frequency). Returns `true` if it succeeded, `false` otherwise.
    - `HalPigpioSetPwmDutycycle` that sets the duty cycle of the PWM for the given GPIO to `dutycycle` by calling [`set_PWM_dutycycle`](https://abyz.me.uk/rpi/pigpio/pdif2.html#set_PWM_dutycycle). Returns `true` if it succeeded, `false` otherwise.
    - `HalPigpioSendTriggerPulse` that sends a pulse of `pulse_length_in_us` µs on the given GPIO by calling [`gpio_trigger`](https://abyz.me.uk/rpi/pigpio/pdif2.html#gpio_trigger). Returns `true` if it succeeded, `false` otherwise.
    - **I2C**
    - `HalPigpioI2cOpen` that opens a connection with the  given I2C device (identified by its `address` on the given `bus`) by calling [`i2c_open`](https://abyz.me.uk/rpi/pigpio/pdif2.html#i2c_open). Returns `true` and the handle if it succeeded, `false` and an error code otherwise.
    - `HalPigpioI2cClose` that closes the connection with the given I2C device by calling [`i2c_close`](https://abyz.me.uk/rpi/pigpio/pdif2.html#i2c_close). Returns `true` if it succeeded, `false` otherwise.
    - `HalPigpioI2cReadByteData` that reads a byte from the given register by calling [`i2c_read_byte_data`](https://abyz.me.uk/rpi/pigpio/pdif2.html#i2c_read_byte_data). Returns `true` and the byte read if it succeeded, `false` and an error code otherwise.
    - `HalPigpioI2cReadWordData` that reads a word (16bits) from the given register by calling [`i2c_read_word_data`](https://abyz.me.uk/rpi/pigpio/pdif2.html#i2c_read_word_data). Returns `true` and the value read if it succeeded, `false` and an error code otherwise.
    - `HalPigpioI2cReadBlockData` that reads `length` bytes from the given register by calling [`i2c_read_i2c_block_data`](https://abyz.me.uk/rpi/pigpio/pdif2.html#i2c_read_i2c_block_data). Returns `true` and an array of the bytes read if it succeeded, `false` and an error code otherwise.
    - `HalPigpioI2cWriteByteData` that writes a byte to the given register by calling [`i2c_write_byte_data`](https://abyz.me.uk/rpi/pigpio/pdif2.html#i2c_write_byte_data). Returns `true` if it succeeded, `false` otherwise.
    - `HalPigpioI2cWriteWordData` that writes a word (16bits) to the given register by calling [`i2c_write_word_data`](https://abyz.me.uk/rpi/pigpio/pdif2.html#i2c_write_word_data). Returns `true` if it succeeded, `false` otherwise.
    - `HalPigpioI2cWriteBlockData` that writes `length` bytes to the given register by calling [`i2c_write_i2c_block_data`](https://abyz.me.uk/rpi/pigpio/pdif2.html#i2c_write_i2c_block_data). Returns `true` if it succeeded, `false` otherwise.
    - **IMU**
    - `HalPigpioI2cImuReading` that starts the given IMU's readings if `is_imu_ready` is true, stops them otherwise. 

## Interfaces

### Topics

N/A

## Dependencies

### Internal

N/A

### External

- `ament_cmake`
- `rclcpp`
- `rosidl_default_generators`
- `rosidl_default_runtime`
- `rosidl_interface_packages`

