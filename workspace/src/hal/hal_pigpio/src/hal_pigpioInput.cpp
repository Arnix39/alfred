#include "hal_pigpio.hpp"

void Pigpio::readGpio(const std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioReadGpio::Request> request,
                      std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioReadGpio::Response> response)
{
    response->level = gpio_read(pigpioHandle, request->gpio_id);
    if (response->level != PI_BAD_GPIO)
    {
        response->has_succeeded = true;
    }
    else
    {
        response->has_succeeded = false;
        RCLCPP_ERROR(get_logger(),"Failed to read GPIO %u!", request->gpio_id);
    }
}

void Pigpio::gpioEdgeChangeCallback(int handle, unsigned gpioId, unsigned edgeChangeType, uint32_t timeSinceBoot_us)
{
    (void)handle;

    auto edgeChange = hal_pigpio_interfaces::msg::HalPigpioEdgeChange();

    edgeChange.gpio_id = gpioId;
    edgeChange.edge_change_type = edgeChangeType;
    edgeChange.time_since_boot_us = timeSinceBoot_us;

    gpioEdgeChangePub->publish(edgeChange);
}

// This is to pass the pointer of the callback function gpioEdgeChangeCallback to the pigpio library API
void Pigpio::c_gpioEdgeChangeCallback(int handle, unsigned gpioId, unsigned edgeChangeType, uint32_t timeSinceBoot_us, void *userData)
{
    Pigpio *object = reinterpret_cast<Pigpio *>(userData);
    object->gpioEdgeChangeCallback(handle, gpioId, edgeChangeType, timeSinceBoot_us);
}

void Pigpio::setCallback(const std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioSetCallback::Request> request,
                         std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioSetCallback::Response> response)
{
    response->callback_id = callback_ex(pigpioHandle, request->gpio_id, request->edge_change_type, c_gpioEdgeChangeCallback, reinterpret_cast<void *>(this));
    if (response->callback_id >= 0)
    {
        response->has_succeeded = true;
        callbackList.push_back((uint)response->callback_id);
        RCLCPP_INFO(get_logger(),"Callback for GPIO %u configured.", request->gpio_id);
    }
    else
    {
        response->has_succeeded = false;
        RCLCPP_ERROR(get_logger(),"Failed to configure callback for GPIO %u!", request->gpio_id);
    }
}

void Pigpio::publishEncoderCount(void)
{
    auto encoderCount = hal_pigpio_interfaces::msg::HalPigpioEncoderCount();

    if (motors.size() != 0)
    {
        for (Motor &motor : motors)
        {
            encoderCount.motor_id = motor.id;
            encoderCount.encoder_count = motor.encoderCount;

            gpioEncoderCountPub->publish(encoderCount);
        }
    }
}

void Pigpio::gpioEncoderEdgeChangeCallback(int handle, unsigned gpioId, unsigned edgeChangeType, uint32_t timeSinceBoot_us)
{
    (void)handle;
    (void)edgeChangeType;
    (void)timeSinceBoot_us;

    for (Motor &motor : motors)
    {
        if (find(motor.gpios.begin(), motor.gpios.end(), gpioId) != motor.gpios.end())
        {
            if(motor.isDirectionForward)
            {
                ++motor.encoderCount;
            }
            else
            {
                --motor.encoderCount;
            }
        }
    }
}

// This is to pass the pointer of the callback function gpioEncoderEdgeChangeCallback to the pigpio library API
void Pigpio::c_gpioEncoderEdgeChangeCallback(int handle, unsigned gpioId, unsigned edgeChangeType, uint32_t timeSinceBoot_us, void *userData)
{
    Pigpio *object = reinterpret_cast<Pigpio *>(userData);
    object->gpioEncoderEdgeChangeCallback(handle, gpioId, edgeChangeType, timeSinceBoot_us);
}

void Pigpio::setEncoderCallback(const std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioSetEncoderCallback::Request> request,
                                std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioSetEncoderCallback::Response> response)
{
    response->callback_id = callback_ex(pigpioHandle, request->gpio_id, request->edge_change_type, c_gpioEncoderEdgeChangeCallback, reinterpret_cast<void *>(this));
    if (response->callback_id >= 0)
    {
        auto motorIndex = find_if(motors.begin(), motors.end(), [request](Motor motor) { return motor.id == request->motor_id; });
        if ( motorIndex != motors.end())
        {
            motors.at(motorIndex - motors.begin()).gpios.push_back(request->gpio_id);
        }
        else
        {
            Motor motor = {request->motor_id, {request->gpio_id}, 0, true};
            motors.push_back(motor);
        }

        response->has_succeeded = true;
        callbackList.push_back((uint)response->callback_id);
        RCLCPP_INFO(get_logger(),"Encoder callback for GPIO %u configured.", request->gpio_id);
    }
    else
    {
        response->has_succeeded = false;
        RCLCPP_ERROR(get_logger(),"Failed to configure encoder callback for GPIO %u!", request->gpio_id);
    }
}

void Pigpio::setMotorDirection(const std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioSetMotorDirection::Request> request,
                               std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioSetMotorDirection::Response> response)
{
    (void)response;

    auto motorIndex = find_if(motors.begin(), motors.end(), [request](Motor motor) { return motor.id == request->motor_id; });
    if ( motorIndex != motors.end())
    {
        motors.at(motorIndex - motors.begin()).isDirectionForward = request->is_direction_forward;
    }
    else
    {
        RCLCPP_ERROR(get_logger(),"Failed to set motor direction for motor %u!", request->motor_id);
    }
}