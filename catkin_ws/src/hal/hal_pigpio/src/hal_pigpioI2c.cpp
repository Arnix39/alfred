#include "hal_pigpioI2c.hpp"

// Services headers (generated)
#include "hal_pigpio/hal_pigpioGetHandle.h"

PigpioI2c::PigpioI2c(ros::NodeHandle *node) : getPigpioHandleClient(node->serviceClient<hal_pigpio::hal_pigpioGetHandle>("hal_pigpioGetHandle"))
{
    hal_pigpio::hal_pigpioGetHandle pigpioHandleRequest;
    getPigpioHandleClient.call(pigpioHandleRequest);
    pigpioHandle = pigpioHandleRequest.response.handle;

    i2cOpenService = node->advertiseService("hal_pigpioI2cOpen", &PigpioI2c::i2cOpen, this);
    i2cCloseService = node->advertiseService("hal_pigpioI2cClose", &PigpioI2c::i2cClose, this);
}

bool PigpioI2c::i2cOpen(hal_pigpio::hal_pigpioI2cOpen::Request &req,
                        hal_pigpio::hal_pigpioI2cOpen::Response &res)
{
    res.handle = i2c_open(pigpioHandle, req.bus, req.address, 0);
    if (res.handle >= 0)
    {
        res.hasSucceeded = true;
        ROS_INFO("I2C bus %u open for device %u.", req.bus, req.address);
    }
    else
    {
        res.hasSucceeded = false;
        ROS_ERROR("Failed to open I2C bus %u for device %u.", req.bus, req.address);
    }
    return true;
}

bool PigpioI2c::i2cClose(hal_pigpio::hal_pigpioI2cClose::Request &req,
                         hal_pigpio::hal_pigpioI2cClose::Response &res)
{
    if (i2c_close(pigpioHandle, req.handle) == 0)
    {
        res.hasSucceeded = true;
        ROS_INFO("I2C device with handle %u closed.", req.handle);
    }
    else
    {
        res.hasSucceeded = false;
        ROS_ERROR("Failed to close I2C device with handle %u.", req.handle);
    }
    return true;
}

bool PigpioI2c::i2cReadByteData(hal_pigpio::hal_pigpioI2cReadByteData::Request &req,
                                hal_pigpio::hal_pigpioI2cReadByteData::Response &res)
{
    res.value = i2c_read_byte_data(pigpioHandle, req.handle, req.deviceRegister);
    if (res.value >= 0)
    {
        res.hasSucceeded = true;
        ROS_INFO("Successfuly read register %u on I2C device %u.", req.deviceRegister, req.handle);
    }
    else
    {
        res.hasSucceeded = false;
        ROS_ERROR("Failed to read register %u on I2C device %u.", req.deviceRegister, req.handle);
    }
    return true;
}

bool PigpioI2c::i2cWriteByteData(hal_pigpio::hal_pigpioI2cWriteByteData::Request &req,
                                 hal_pigpio::hal_pigpioI2cWriteByteData::Response &res)
{
    if (i2c_write_byte_data(pigpioHandle, req.handle, req.deviceRegister, req.value) == 0)
    {
        res.hasSucceeded = true;
        ROS_INFO("Successfuly wrote register %u on I2C device %u.", req.deviceRegister, req.handle);
    }
    else
    {
        res.hasSucceeded = false;
        ROS_ERROR("Failed to write register %u on I2C device %u.", req.deviceRegister, req.handle);
    }
    return true;
}