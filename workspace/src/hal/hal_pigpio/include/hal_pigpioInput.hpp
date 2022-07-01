#ifndef HAL_PIGPIO_INPUT
#define HAL_PIGPIO_INPUT

#include "rclcpp/rclcpp.hpp"

// Pigpio library
#include <pigpiod_if2.h>

// Services and messages headers (generated)
#include "hal_pigpio/srv/hal_pigpio_read_gpio.hpp"
#include "hal_pigpio/srv/hal_pigpio_set_callback.hpp"
#include "hal_pigpio/srv/hal_pigpio_set_encoder_callback.hpp"
#include "hal_pigpio/srv/hal_pigpio_set_motor_direction.hpp"
#include "hal_pigpio/msg/hal_pigpio_edge_change.hpp"
#include "hal_pigpio/msg/hal_pigpio_encoder_count.hpp"


struct Motor
{
    uint8_t id;
    std::vector<unsigned> gpios;
    int32_t encoderCount;
    bool isDirectionForward;
};

class PigpioInput
{
private:
    int pigpioHandle;
    std::shared_ptr<rclcpp::Node> halPigpioNode;
    rclcpp::Publisher gpioEdgeChangePub;
    rclcpp::Publisher gpioEncoderCountPub;
    rclcpp::ServiceServer readGpioService;
    rclcpp::ServiceServer setCallbackService;
    rclcpp::ServiceServer setEncoderCallbackService;
    rclcpp::ServiceServer setMotorDirectionService;
    std::vector<uint> callbackList;
    std::vector<Motor> motors;
    inline void gpioEdgeChangeCallback(int handle, unsigned gpioId, unsigned edgeChangeType, uint32_t timeSinceBoot_us);
    static void c_gpioEdgeChangeCallback(int handle, unsigned gpioId, unsigned edgeChangeType, uint32_t timeSinceBoot_us, void *userData);
    inline void gpioEncoderEdgeChangeCallback(int handle, unsigned gpioId, unsigned edgeChangeType, uint32_t timeSinceBoot_us);
    static void c_gpioEncoderEdgeChangeCallback(int handle, unsigned gpioId, unsigned edgeChangeType, uint32_t timeSinceBoot_us, void *userData);

public:
    PigpioInput(std::shared_ptr<rclcpp::Node> node, int pigpioHandle);
    ~PigpioInput();
    bool readGpio(hal_pigpio::hal_pigpioReadGpio::Request &req,
                  hal_pigpio::hal_pigpioReadGpio::Response &res);
    bool setCallback(hal_pigpio::hal_pigpioSetCallback::Request &req,
                     hal_pigpio::hal_pigpioSetCallback::Response &res);
    bool setEncoderCallback(hal_pigpio::hal_pigpioSetEncoderCallback::Request &req,
                            hal_pigpio::hal_pigpioSetEncoderCallback::Response &res);
    bool setMotorDirection(hal_pigpio::hal_pigpioSetMotorDirection::Request &req,
                           hal_pigpio::hal_pigpioSetMotorDirection::Response &res);
    void publishEncoderCount(const rclcpp::TimerEvent &timerEvent);
};

#endif