#ifndef HAL_PIGPIO_I2C
#define HAL_PIGPIO_I2C

#include "rclcpp/rclcpp.hpp"

// Pigpio library
#include <pigpiod_if2.h>

// Services and messages headers (generated)
#include "hal_pigpio/srv/hal_pigpio_i2c_open.hpp"
#include "hal_pigpio/srv/hal_pigpio_i2c_close.hpp"
#include "hal_pigpio/srv/hal_pigpio_i2c_read_byte_data.hpp"
#include "hal_pigpio/srv/hal_pigpio_i2c_read_word_data.hpp"
#include "hal_pigpio/srv/hal_pigpio_i2c_read_block_data.hpp"
#include "hal_pigpio/srv/hal_pigpio_i2c_write_byte_data.hpp"
#include "hal_pigpio/srv/hal_pigpio_i2c_write_word_data.hpp"
#include "hal_pigpio/srv/hal_pigpio_i2c_write_block_data.hpp"

#define I2C_BUFFER_MAX_BYTES 32

class PigpioI2c
{
private:
    ros::ServiceServer i2cOpenService;
    ros::ServiceServer i2cCloseService;
    ros::ServiceServer i2cWriteByteDataService;
    ros::ServiceServer i2cWriteWordDataService;
    ros::ServiceServer i2cWriteBlockDataService;
    ros::ServiceServer i2cReadByteDataService;
    ros::ServiceServer i2cReadWordDataService;
    ros::ServiceServer i2cReadBlockDataService;
    int pigpioHandle;

public:
    PigpioI2c(ros::NodeHandle *node, int pigpioHandle);
    ~PigpioI2c() = default;
    bool i2cOpen(hal_pigpio::hal_pigpioI2cOpen::Request &req,
                 hal_pigpio::hal_pigpioI2cOpen::Response &res);
    bool i2cClose(hal_pigpio::hal_pigpioI2cClose::Request &req,
                  hal_pigpio::hal_pigpioI2cClose::Response &res);
    bool i2cReadByteData(hal_pigpio::hal_pigpioI2cReadByteData::Request &req,
                         hal_pigpio::hal_pigpioI2cReadByteData::Response &res);
    bool i2cReadWordData(hal_pigpio::hal_pigpioI2cReadWordData::Request &req,
                         hal_pigpio::hal_pigpioI2cReadWordData::Response &res);
    bool i2cReadBlockData(hal_pigpio::hal_pigpioI2cReadBlockData::Request &req,
                          hal_pigpio::hal_pigpioI2cReadBlockData::Response &res);
    bool i2cWriteByteData(hal_pigpio::hal_pigpioI2cWriteByteData::Request &req,
                          hal_pigpio::hal_pigpioI2cWriteByteData::Response &res);
    bool i2cWriteWordData(hal_pigpio::hal_pigpioI2cWriteWordData::Request &req,
                          hal_pigpio::hal_pigpioI2cWriteWordData::Response &res);
    bool i2cWriteBlockData(hal_pigpio::hal_pigpioI2cWriteBlockData::Request &req,
                           hal_pigpio::hal_pigpioI2cWriteBlockData::Response &res);
};

#endif