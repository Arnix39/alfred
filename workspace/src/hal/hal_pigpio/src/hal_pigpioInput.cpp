#include "hal_pigpioInput.hpp"

PigpioInput::PigpioInput(rclcpp::NodeHandle *node, int pigpioHandle) : pigpioHandle(pigpioHandle),
                                                                    gpioEdgeChangePub(node->advertise<hal_pigpio::hal_pigpioEdgeChangeMsg>("gpioEdgeChange", 1000)),
                                                                    gpioEncoderCountPub(node->advertise<hal_pigpio::hal_pigpioEncoderCountMsg>("hal_pigpioEncoderCount", 1000)),
                                                                    readGpioService(node->advertiseService("hal_pigpioReadGpio", &PigpioInput::readGpio, this)),
                                                                    setCallbackService(node->advertiseService("hal_pigpioSetCallback", &PigpioInput::setCallback, this)),
                                                                    setEncoderCallbackService(node->advertiseService("hal_pigpioSetEncoderCallback", &PigpioInput::setEncoderCallback, this)),
                                                                    setMotorDirectionService(node->advertiseService("hal_pigpioSetMotorDirection", &PigpioInput::setMotorDirection, this)),
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

bool PigpioInput::readGpio(hal_pigpio::hal_pigpioReadGpio::Request &req,
                           hal_pigpio::hal_pigpioReadGpio::Response &res)
{
    res.level = gpio_read(pigpioHandle, req.gpioId);
    if (res.level != PI_BAD_GPIO)
    {
        res.hasSucceeded = true;
    }
    else
    {
        res.hasSucceeded = false;
        RCLCPP_ERROR("Failed to read GPIO %u!", req.gpioId);
    }
    return true;
}

void PigpioInput::gpioEdgeChangeCallback(int handle, unsigned gpioId, unsigned edgeChangeType, uint32_t timeSinceBoot_us)
{
    hal_pigpio::hal_pigpioEdgeChangeMsg edgeChangeMsg;

    edgeChangeMsg.gpioId = gpioId;
    edgeChangeMsg.edgeChangeType = edgeChangeType;
    edgeChangeMsg.timeSinceBoot_us = timeSinceBoot_us;

    gpioEdgeChangePub.publish(edgeChangeMsg);
}

// This is to pass the pointer of the callback function gpioEdgeChangeCallback to the pigpio library API
void PigpioInput::c_gpioEdgeChangeCallback(int handle, unsigned gpioId, unsigned edgeChangeType, uint32_t timeSinceBoot_us, void *userData)
{
    PigpioInput *object = reinterpret_cast<PigpioInput *>(userData);
    object->gpioEdgeChangeCallback(handle, gpioId, edgeChangeType, timeSinceBoot_us);
}

bool PigpioInput::setCallback(hal_pigpio::hal_pigpioSetCallback::Request &req,
                              hal_pigpio::hal_pigpioSetCallback::Response &res)
{
    res.callbackId = callback_ex(pigpioHandle, req.gpioId, req.edgeChangeType, c_gpioEdgeChangeCallback, reinterpret_cast<void *>(this));
    if (res.callbackId >= 0)
    {
        res.hasSucceeded = true;
        callbackList.push_back((uint)res.callbackId);
        RCLCPP_INFO("Callback for GPIO %u configured.", req.gpioId);
    }
    else
    {
        res.hasSucceeded = false;
        RCLCPP_ERROR("Failed to configure callback for GPIO %u!", req.gpioId);
    }
    return true;
}

void PigpioInput::publishEncoderCount(const rclcpp::TimerEvent &timerEvent)
{
    hal_pigpio::hal_pigpioEncoderCountMsg encoderCountMsg;

    if (motors.size() != 0)
    {
        for (Motor &motor : motors)
        {
            encoderCountMsg.motorId = motor.id;
            encoderCountMsg.encoderCount = motor.encoderCount;

            gpioEncoderCountPub.publish(encoderCountMsg);
        }
    }
}

void PigpioInput::gpioEncoderEdgeChangeCallback(int handle, unsigned gpioId, unsigned edgeChangeType, uint32_t timeSinceBoot_us)
{
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

bool PigpioInput::setEncoderCallback(hal_pigpio::hal_pigpioSetEncoderCallback::Request &req,
                                     hal_pigpio::hal_pigpioSetEncoderCallback::Response &res)
{
    res.callbackId = callback_ex(pigpioHandle, req.gpioId, req.edgeChangeType, c_gpioEncoderEdgeChangeCallback, reinterpret_cast<void *>(this));
    if (res.callbackId >= 0)
    {
        auto motorIndex = find_if(motors.begin(), motors.end(), [&req](Motor motor) { return motor.id == req.motorId; });
        if ( motorIndex != motors.end())
        {
            motors.at(motorIndex - motors.begin()).gpios.push_back(req.gpioId);
        }
        else
        {
            Motor motor = {req.motorId, {req.gpioId}, 0};
            motors.push_back(motor);
        }

        res.hasSucceeded = true;
        callbackList.push_back((uint)res.callbackId);
        RCLCPP_INFO("Encoder callback for GPIO %u configured.", req.gpioId);
    }
    else
    {
        res.hasSucceeded = false;
        RCLCPP_ERROR("Failed to configure encoder callback for GPIO %u!", req.gpioId);
    }
    return true;
}

bool PigpioInput::setMotorDirection(hal_pigpio::hal_pigpioSetMotorDirection::Request &req,
                                    hal_pigpio::hal_pigpioSetMotorDirection::Response &res)
{
    auto motorIndex = find_if(motors.begin(), motors.end(), [&req](Motor motor) { return motor.id == req.motorId; });
    if ( motorIndex != motors.end())
    {
        motors.at(motorIndex - motors.begin()).isDirectionForward = req.isDirectionForward;
    }
    else
    {
        RCLCPP_ERROR("Failed to set motor direction for motor %u!", req.motorId);
    }
    return true;
}