#ifndef HAL_PIGPIO_MOCK
#define HAL_PIGPIO_MOCK

#include <stdint.h>

#define PI_BAD_GPIO -1
#define PI_NOT_PERMITTED -2
#define PI_BAD_USER_GPIO -3
#define PI_INPUT 0
#define PI_OUTPUT 1
#define PI_LOW 0
#define PI_HIGH 1
#define PI_PUD_OFF 0
#define PI_PUD_DOWN 1
#define PI_PUD_UP 2

typedef void (*CBFuncEx_t) (int pi, unsigned user_gpio, unsigned level, uint32_t tick, void * userdata);

int i2c_open(int pi, unsigned i2c_bus, unsigned i2c_addr, unsigned i2c_flags);
int i2c_close(int pi, unsigned handle);
int i2c_read_byte_data(int pi, unsigned handle, unsigned i2c_reg);
int i2c_read_word_data(int pi, unsigned handle, unsigned i2c_reg);
int i2c_read_i2c_block_data(int pi, unsigned handle, unsigned i2c_reg, char *buf, unsigned count);
int i2c_write_byte_data(int pi, unsigned handle, unsigned i2c_reg, unsigned bVal);
int i2c_write_word_data(int pi, unsigned handle, unsigned i2c_reg, unsigned wVal);
int i2c_write_i2c_block_data(int pi, unsigned handle, unsigned i2c_reg, char *buf, unsigned count);

int pigpio_start(const char *addrStr, const char *portStr);
void pigpio_stop(int pi);
int set_mode(int pi, unsigned gpio, unsigned mode);
int get_mode(int pi, unsigned gpio);
int set_pull_up_down(int pi, unsigned gpio, unsigned pud);

int callback_cancel(unsigned callback_id);
int gpio_read(int pi, unsigned gpio);
int callback_ex(int pi, unsigned user_gpio, unsigned edge, CBFuncEx_t f, void *userdata);

int set_PWM_dutycycle(int pi, unsigned user_gpio, unsigned dutycycle);
int set_PWM_frequency(int pi, unsigned user_gpio, unsigned frequency);
int gpio_write(int pi, unsigned gpio, unsigned level);
int gpio_trigger(int pi, unsigned user_gpio, unsigned pulseLen, unsigned level);

#endif