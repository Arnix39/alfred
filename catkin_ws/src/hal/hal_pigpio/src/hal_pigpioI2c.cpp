#include "hal_pigpioI2c.hpp"

// Services headers (generated)
#include "hal_pigpio/hal_pigpioGetHandle.h"

PigpioI2c::PigpioI2c(ros::NodeHandle *node, int pigpioHandle) : pigpioHandle(pigpioHandle),
                                                                getPigpioHandleClient(node->serviceClient<hal_pigpio::hal_pigpioGetHandle>("hal_pigpioGetHandle")),
                                                                i2cOpenService(node->advertiseService("hal_pigpioI2cOpen", &PigpioI2c::i2cOpen, this)),
                                                                i2cCloseService(node->advertiseService("hal_pigpioI2cClose", &PigpioI2c::i2cClose, this)),
                                                                i2cReadByteDataService(node->advertiseService("hal_pigpioI2cReadByteData", &PigpioI2c::i2cReadByteData, this)),
                                                                i2cReadWordDataService(node->advertiseService("hal_pigpioI2cReadWordData", &PigpioI2c::i2cReadWordData, this)),
                                                                i2cWriteByteDataService(node->advertiseService("hal_pigpioI2cWriteByteData", &PigpioI2c::i2cWriteByteData, this)),
                                                                i2cWriteWordDataService(node->advertiseService("hal_pigpioI2cWriteWordData", &PigpioI2c::i2cWriteWordData, this)),
                                                                i2cWriteBlockDataService(node->advertiseService("hal_pigpioI2cWriteBlockData", &PigpioI2c::i2cWriteBlockData, this))
{
}

bool PigpioI2c::i2cOpen(hal_pigpio::hal_pigpioI2cOpen::Request &req,
                        hal_pigpio::hal_pigpioI2cOpen::Response &res)
{
    res.handle = i2c_open(pigpioHandle, req.bus, req.address, 0);
    if (res.handle >= 0)
    {
        res.hasSucceeded = true;
        ROS_INFO("I2C bus %u open for device %u with handle %u.", req.bus, req.address, res.handle);
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
    int result = i2c_read_byte_data(pigpioHandle, req.handle, req.deviceRegister);
    if (result >= 0)
    {
        res.value = (uint8_t)result;
        res.hasSucceeded = true;
        ROS_INFO("Successfuly read %u in register %u on device with handle %u.", res.value, req.deviceRegister, req.handle);
    }
    else
    {
        res.value = 0;
        res.hasSucceeded = false;
        ROS_ERROR("Failed to read register %u on device with handle %u.", req.deviceRegister, req.handle);
    }
    return true;
}

bool PigpioI2c::i2cReadWordData(hal_pigpio::hal_pigpioI2cReadWordData::Request &req,
                                hal_pigpio::hal_pigpioI2cReadWordData::Response &res)
{
    int result = i2c_read_word_data(pigpioHandle, req.handle, req.deviceRegister);
    if (result >= 0)
    {
        res.value = (uint16_t)result;
        res.hasSucceeded = true;
        ROS_INFO("Successfuly read %u in register %u on device with handle %u.", res.value, req.deviceRegister, req.handle);
    }
    else
    {
        res.value = 0;
        res.hasSucceeded = false;
        ROS_ERROR("Failed to read register %u on device with handle %u.", req.deviceRegister, req.handle);
    }
    return true;
}

bool PigpioI2c::i2cWriteByteData(hal_pigpio::hal_pigpioI2cWriteByteData::Request &req,
                                 hal_pigpio::hal_pigpioI2cWriteByteData::Response &res)
{
    if (i2c_write_byte_data(pigpioHandle, req.handle, req.deviceRegister, req.value) == 0)
    {
        res.hasSucceeded = true;
        ROS_INFO("Successfuly wrote %u in register %u on device with handle %u.", req.value, req.deviceRegister, req.handle);
    }
    else
    {
        res.hasSucceeded = false;
        ROS_ERROR("Failed to write register %u on device with handle %u.", req.deviceRegister, req.handle);
    }
    return true;
}

bool PigpioI2c::i2cWriteWordData(hal_pigpio::hal_pigpioI2cWriteWordData::Request &req,
                                 hal_pigpio::hal_pigpioI2cWriteWordData::Response &res)
{
    if (i2c_write_word_data(pigpioHandle, req.handle, req.deviceRegister, req.value) == 0)
    {
        res.hasSucceeded = true;
        ROS_INFO("Successfuly wrote %u in register %u on device with handle %u.", req.value, req.deviceRegister, req.handle);
    }
    else
    {
        res.hasSucceeded = false;
        ROS_ERROR("Failed to write register %u on device with handle %u.", req.deviceRegister, req.handle);
    }
    return true;
}

bool PigpioI2c::i2cWriteBlockData(hal_pigpio::hal_pigpioI2cWriteBlockData::Request &req,
                                  hal_pigpio::hal_pigpioI2cWriteBlockData::Response &res)
{
    char dataBlock[16];

    for (uint8_t byte = 0; byte < req.length; byte++)
    {
        dataBlock[byte] = (char)(req.dataBlock[byte]);
    }

    if (i2c_write_block_data(pigpioHandle, req.handle, req.deviceRegister, dataBlock, req.length) == 0)
    {
        res.hasSucceeded = true;
        ROS_INFO("Successfuly wrote data block in register %u on device with handle %u.", req.deviceRegister, req.handle);
    }
    else
    {
        res.hasSucceeded = false;
        ROS_ERROR("Failed to write data block in register %u on device with handle %u.", req.deviceRegister, req.handle);
    }
    return true;
}