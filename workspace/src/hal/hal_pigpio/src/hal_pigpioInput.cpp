#include "hal_pigpioInput.hpp"

PigpioInput::PigpioInput(std::shared_ptr<rclcpp::Node> node, int pigpioHandle) :    pigpioHandle(pigpioHandle),
                                                                                    halPigpioNode(node),
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

void PigpioInput::readGpio(hal_pigpio::hal_pigpioReadGpio::Request request,
                           hal_pigpio::hal_pigpioReadGpio::Response response)
{
    response->level = gpio_read(pigpioHandle, request->gpioId);
    if (response->level != PI_BAD_GPIO)
    {
        response->has_succeeded = true;
    }
    else
    {
        response->has_succeeded = false;
        RCLCPP_ERROR(halPigpioNode->get_logger(),"Failed to read GPIO %u!", request->gpioId);
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

void PigpioInput::setCallback(hal_pigpio::hal_pigpioSetCallback::Request request,
                              hal_pigpio::hal_pigpioSetCallback::Response response)
{
    response->callbackId = callback_ex(pigpioHandle, request->gpioId, request->edgeChangeType, c_gpioEdgeChangeCallback, reinterpret_cast<void *>(this));
    if (response->callbackId >= 0)
    {
        response->has_succeeded = true;
        callbackList.push_back((uint)response->callbackId);
        RCLCPP_INFO(halPigpioNode->get_logger(),"Callback for GPIO %u configured.", request->gpioId);
    }
    else
    {
        response->has_succeeded = false;
        RCLCPP_ERROR(halPigpioNode->get_logger(),"Failed to configure callback for GPIO %u!", request->gpioId);
    }
    return true;
}

void PigpioInput::publishEncoderCount(void)
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

void PigpioInput::setEncoderCallback(hal_pigpio::hal_pigpioSetEncoderCallback::Request request,
                                     hal_pigpio::hal_pigpioSetEncoderCallback::Response response)
{
    response->callbackId = callback_ex(pigpioHandle, request->gpioId, request->edgeChangeType, c_gpioEncoderEdgeChangeCallback, reinterpret_cast<void *>(this));
    if (response->callbackId >= 0)
    {
        auto motorIndex = find_if(motors.begin(), motors.end(), [request](Motor motor) { return motor.id == request->motorId; });
        if ( motorIndex != motors.end())
        {
            motors.at(motorIndex - motors.begin()).gpios.push_back(request->gpioId);
        }
        else
        {
            Motor motor = {request->motorId, {request->gpioId}, 0};
            motors.push_back(motor);
        }

        response->has_succeeded = true;
        callbackList.push_back((uint)response->callbackId);
        RCLCPP_INFO(halPigpioNode->get_logger(),"Encoder callback for GPIO %u configured.", request->gpioId);
    }
    else
    {
        response->has_succeeded = false;
        RCLCPP_ERROR(halPigpioNode->get_logger(),"Failed to configure encoder callback for GPIO %u!", request->gpioId);
    }
    return true;
}

void PigpioInput::setMotorDirection(hal_pigpio::hal_pigpioSetMotorDirection::Request request,
                                    hal_pigpio::hal_pigpioSetMotorDirection::Response response)
{
    auto motorIndex = find_if(motors.begin(), motors.end(), [request](Motor motor) { return motor.id == request->motorId; });
    if ( motorIndex != motors.end())
    {
        motors.at(motorIndex - motors.begin()).isDirectionForward = request->isDirectionForward;
    }
    else
    {
        RCLCPP_ERROR(halPigpioNode->get_logger(),"Failed to set motor direction for motor %u!", request->motorId);
    }
    return true;
}