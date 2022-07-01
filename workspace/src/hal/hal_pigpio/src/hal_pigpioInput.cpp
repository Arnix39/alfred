#include "hal_pigpioInput.hpp"

using namespace std::placeholders;

PigpioInput::PigpioInput(std::shared_ptr<rclcpp::Node> node, int pigpioHandle) :    pigpioHandle(pigpioHandle),
                                                                                    halPigpioNode(node),
                                                                                    gpioEdgeChangePub(node->create_publisher<hal_pigpio::msg::HalPigpioEdgeChange>("gpioEdgeChange", 1000)),
                                                                                    gpioEncoderCountPub(node->create_publisher<hal_pigpio::msg::HalPigpioEncoderCount>("hal_pigpioEncoderCount", 1000)),
                                                                                    readGpioService(node->create_service<hal_pigpio::srv::HalPigpioReadGpio>("hal_pigpioReadGpio", std::bind(&PigpioInput::readGpio, this, _1, _2))),
                                                                                    setCallbackService(node->create_service<hal_pigpio::srv::HalPigpioSetCallback>("hal_pigpioSetCallback", std::bind(&PigpioInput::setCallback, this, _1, _2))),
                                                                                    setEncoderCallbackService(node->create_service<hal_pigpio::srv::HalPigpioSetEncoderCallback>("hal_pigpioSetEncoderCallback", std::bind(&PigpioInput::setEncoderCallback, this, _1, _2))),
                                                                                    setMotorDirectionService(node->create_service<hal_pigpio::srv::HalPigpioSetMotorDirection>("hal_pigpioSetMotorDirection", std::bind(&PigpioInput::setMotorDirection, this, _1, _2))),
                                                                                    callbackList({}),
                                                                                    motors({})
{
}

PigpioInput::~PigpioInput()
{
    for (uint callbackId : callbackList)
    {
        callback_cancel(callbackId);
    }
}

void PigpioInput::readGpio(const std::shared_ptr<hal_pigpio::srv::HalPigpioReadGpio::Request> request,
                           std::shared_ptr<hal_pigpio::srv::HalPigpioReadGpio::Response> response)
{
    response->level = gpio_read(pigpioHandle, request->gpio_id);
    if (response->level != PI_BAD_GPIO)
    {
        response->has_succeeded = true;
    }
    else
    {
        response->has_succeeded = false;
        RCLCPP_ERROR(halPigpioNode->get_logger(),"Failed to read GPIO %u!", request->gpio_id);
    }
}

void PigpioInput::gpioEdgeChangeCallback(int handle, unsigned gpioId, unsigned edgeChangeType, uint32_t timeSinceBoot_us)
{
    (void)handle;

    auto edgeChange = hal_pigpio::msg::HalPigpioEdgeChange();

    edgeChange.gpio_id = gpioId;
    edgeChange.edge_change_type = edgeChangeType;
    edgeChange.time_since_boot_us = timeSinceBoot_us;

    gpioEdgeChangePub->publish(edgeChange);
}

// This is to pass the pointer of the callback function gpioEdgeChangeCallback to the pigpio library API
void PigpioInput::c_gpioEdgeChangeCallback(int handle, unsigned gpioId, unsigned edgeChangeType, uint32_t timeSinceBoot_us, void *userData)
{
    PigpioInput *object = reinterpret_cast<PigpioInput *>(userData);
    object->gpioEdgeChangeCallback(handle, gpioId, edgeChangeType, timeSinceBoot_us);
}

void PigpioInput::setCallback(const std::shared_ptr<hal_pigpio::srv::HalPigpioSetCallback::Request> request,
                              std::shared_ptr<hal_pigpio::srv::HalPigpioSetCallback::Response> response)
{
    response->callback_id = callback_ex(pigpioHandle, request->gpio_id, request->edge_change_type, c_gpioEdgeChangeCallback, reinterpret_cast<void *>(this));
    if (response->callback_id >= 0)
    {
        response->has_succeeded = true;
        callbackList.push_back((uint)response->callback_id);
        RCLCPP_INFO(halPigpioNode->get_logger(),"Callback for GPIO %u configured.", request->gpio_id);
    }
    else
    {
        response->has_succeeded = false;
        RCLCPP_ERROR(halPigpioNode->get_logger(),"Failed to configure callback for GPIO %u!", request->gpio_id);
    }
}

void PigpioInput::publishEncoderCount(void)
{
    auto encoderCount = hal_pigpio::msg::HalPigpioEncoderCount();

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

void PigpioInput::gpioEncoderEdgeChangeCallback(int handle, unsigned gpioId, unsigned edgeChangeType, uint32_t timeSinceBoot_us)
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
void PigpioInput::c_gpioEncoderEdgeChangeCallback(int handle, unsigned gpioId, unsigned edgeChangeType, uint32_t timeSinceBoot_us, void *userData)
{
    PigpioInput *object = reinterpret_cast<PigpioInput *>(userData);
    object->gpioEncoderEdgeChangeCallback(handle, gpioId, edgeChangeType, timeSinceBoot_us);
}

void PigpioInput::setEncoderCallback(const std::shared_ptr<hal_pigpio::srv::HalPigpioSetEncoderCallback::Request> request,
                                     std::shared_ptr<hal_pigpio::srv::HalPigpioSetEncoderCallback::Response> response)
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
        RCLCPP_INFO(halPigpioNode->get_logger(),"Encoder callback for GPIO %u configured.", request->gpio_id);
    }
    else
    {
        response->has_succeeded = false;
        RCLCPP_ERROR(halPigpioNode->get_logger(),"Failed to configure encoder callback for GPIO %u!", request->gpio_id);
    }
}

void PigpioInput::setMotorDirection(const std::shared_ptr<hal_pigpio::srv::HalPigpioSetMotorDirection::Request> request,
                                    std::shared_ptr<hal_pigpio::srv::HalPigpioSetMotorDirection::Response> response)
{
    (void)response;

    auto motorIndex = find_if(motors.begin(), motors.end(), [request](Motor motor) { return motor.id == request->motor_id; });
    if ( motorIndex != motors.end())
    {
        motors.at(motorIndex - motors.begin()).isDirectionForward = request->is_direction_forward;
    }
    else
    {
        RCLCPP_ERROR(halPigpioNode->get_logger(),"Failed to set motor direction for motor %u!", request->motor_id);
    }
}