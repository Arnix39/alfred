#include "hal_pigpioI2c.hpp"

using namespace std::placeholders;

PigpioI2c::PigpioI2c(std::shared_ptr<rclcpp::Node> node, int pigpioHandle) :    pigpioHandle(pigpioHandle),
                                                                                halPigpioNode(node),
                                                                                i2cOpenService(node->create_service<hal_pigpio_interfaces::srv::HalPigpioI2cOpen>("hal_pigpioI2cOpen", std::bind(&PigpioI2c::i2cOpen, this, _1, _2))),
                                                                                i2cCloseService(node->create_service<hal_pigpio_interfaces::srv::HalPigpioI2cClose>("hal_pigpioI2cClose", std::bind(&PigpioI2c::i2cClose, this, _1, _2))),
                                                                                i2cReadByteDataService(node->create_service<hal_pigpio_interfaces::srv::HalPigpioI2cReadByteData>("hal_pigpioI2cReadByteData", std::bind(&PigpioI2c::i2cReadByteData, this, _1, _2))),
                                                                                i2cReadWordDataService(node->create_service<hal_pigpio_interfaces::srv::HalPigpioI2cReadWordData>("hal_pigpioI2cReadWordData", std::bind(&PigpioI2c::i2cReadWordData, this, _1, _2))),
                                                                                i2cReadBlockDataService(node->create_service<hal_pigpio_interfaces::srv::HalPigpioI2cReadBlockData>("hal_pigpioI2cReadBlockData", std::bind(&PigpioI2c::i2cReadBlockData, this, _1, _2))),
                                                                                i2cWriteByteDataService(node->create_service<hal_pigpio_interfaces::srv::HalPigpioI2cWriteByteData>("hal_pigpioI2cWriteByteData", std::bind(&PigpioI2c::i2cWriteByteData, this, _1, _2))),
                                                                                i2cWriteWordDataService(node->create_service<hal_pigpio_interfaces::srv::HalPigpioI2cWriteWordData>("hal_pigpioI2cWriteWordData", std::bind(&PigpioI2c::i2cWriteWordData, this, _1, _2))),
                                                                                i2cWriteBlockDataService(node->create_service<hal_pigpio_interfaces::srv::HalPigpioI2cWriteBlockData>("hal_pigpioI2cWriteBlockData", std::bind(&PigpioI2c::i2cWriteBlockData, this, _1, _2)))
{
}

void PigpioI2c::i2cOpen(const std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioI2cOpen::Request> request,
                        std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioI2cOpen::Response> response)
{
    response->handle = i2c_open(pigpioHandle, request->bus, request->address, 0);
    if (response->handle >= 0)
    {
        response->has_succeeded = true;
        RCLCPP_INFO(halPigpioNode->get_logger(),"I2C bus %u open for device %u with handle %u.", request->bus, request->address, response->handle);
    }
    else
    {
        response->has_succeeded = false;
        RCLCPP_ERROR(halPigpioNode->get_logger(),"Failed to open I2C bus %u for device %u.", request->bus, request->address);
    }
}

void PigpioI2c::i2cClose(const std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioI2cClose::Request> request,
                         std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioI2cClose::Response> response)
{
    if (i2c_close(pigpioHandle, request->handle) == 0)
    {
        response->has_succeeded = true;
        RCLCPP_INFO(halPigpioNode->get_logger(),"I2C device with handle %u closed.", request->handle);
    }
    else
    {
        response->has_succeeded = false;
        RCLCPP_ERROR(halPigpioNode->get_logger(),"Failed to close I2C device with handle %u.", request->handle);
    }
}

void PigpioI2c::i2cReadByteData(const std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioI2cReadByteData::Request> request,
                                std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioI2cReadByteData::Response> response)
{
    int result = i2c_read_byte_data(pigpioHandle, request->handle, request->device_register);
    if (result >= 0)
    {
        response->value = static_cast<uint8_t>(result);
        response->has_succeeded = true;
    }
    else
    {
        response->value = 0;
        response->has_succeeded = false;
        RCLCPP_ERROR(halPigpioNode->get_logger(),"Failed to read register %u on device with handle %u.", request->device_register, request->handle);
    }
}

void PigpioI2c::i2cReadWordData(const std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioI2cReadWordData::Request> request,
                                std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioI2cReadWordData::Response> response)
{
    int result = i2c_read_word_data(pigpioHandle, request->handle, request->device_register);
    if (result >= 0)
    {
        response->value = static_cast<uint16_t>(result);
        response->has_succeeded = true;
    }
    else
    {
        response->value = 0;
        response->has_succeeded = false;
        RCLCPP_ERROR(halPigpioNode->get_logger(),"Failed to read register %u on device with handle %u.", request->device_register, request->handle);
    }
}

void PigpioI2c::i2cReadBlockData(const std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioI2cReadBlockData::Request> request,
                                 std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioI2cReadBlockData::Response> response)
{
    char buffer[I2C_BUFFER_MAX_BYTES];
    int result = i2c_read_i2c_block_data(pigpioHandle, request->handle, request->device_register, buffer, request->length);
    if (result > 0)
    {
        response->has_succeeded = true;

        for (uint8_t index = 0; index < result; index++)
        {
            response->data_block.push_back(buffer[index]);
        }

        RCLCPP_INFO(halPigpioNode->get_logger(),"Successfuly read data block from register %u on device with handle %u.", request->device_register, request->handle);
    }
    else
    {
        response->has_succeeded = false;
        RCLCPP_ERROR(halPigpioNode->get_logger(),"Failed to read register %u on device with handle %u.", request->device_register, request->handle);
    }
}

void PigpioI2c::i2cWriteByteData(const std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioI2cWriteByteData::Request> request,
                                 std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioI2cWriteByteData::Response> response)
{
    if (i2c_write_byte_data(pigpioHandle, request->handle, request->device_register, request->value) == 0)
    {
        response->has_succeeded = true;
    }
    else
    {
        response->has_succeeded = false;
        RCLCPP_ERROR(halPigpioNode->get_logger(),"Failed to write register %u on device with handle %u.", request->device_register, request->handle);
    }
}

void PigpioI2c::i2cWriteWordData(const std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioI2cWriteWordData::Request> request,
                                 std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioI2cWriteWordData::Response> response)
{
    if (i2c_write_word_data(pigpioHandle, request->handle, request->device_register, request->value) == 0)
    {
        response->has_succeeded = true;
    }
    else
    {
        response->has_succeeded = false;
        RCLCPP_ERROR(halPigpioNode->get_logger(),"Failed to write register %u on device with handle %u.", request->device_register, request->handle);
    }
}

void PigpioI2c::i2cWriteBlockData(const std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioI2cWriteBlockData::Request> request,
                                  std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioI2cWriteBlockData::Response> response)
{
    char dataBlock[I2C_BUFFER_MAX_BYTES];

    for (uint8_t byte = 0; byte < request->length; byte++)
    {
        dataBlock[byte] = (char)(request->data_block[byte]);
    }

    if (i2c_write_i2c_block_data(pigpioHandle, request->handle, request->device_register, dataBlock, request->length) == 0)
    {
        response->has_succeeded = true;
    }
    else
    {
        response->has_succeeded = false;
        RCLCPP_ERROR(halPigpioNode->get_logger(),"Failed to write data block in register %u on device with handle %u.", request->device_register, request->handle);
    }
}