#ifndef HAL_PIGPIO_INPUT
#define HAL_PIGPIO_INPUT

#include "rclcpp/rclcpp.hpp"

// Pigpio library
#include <pigpiod_if2.h>

// Services and messages headers (generated)
#include "hal_pigpio_interfaces/srv/hal_pigpio_read_gpio.hpp"
#include "hal_pigpio_interfaces/srv/hal_pigpio_set_callback.hpp"
#include "hal_pigpio_interfaces/srv/hal_pigpio_set_encoder_callback.hpp"
#include "hal_pigpio_interfaces/srv/hal_pigpio_set_motor_direction.hpp"
#include "hal_pigpio_interfaces/msg/hal_pigpio_edge_change.hpp"
#include "hal_pigpio_interfaces/msg/hal_pigpio_encoder_count.hpp"


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
    rclcpp::Publisher<hal_pigpio_interfaces::msg::HalPigpioEdgeChange>::SharedPtr gpioEdgeChangePub;
    rclcpp::Publisher<hal_pigpio_interfaces::msg::HalPigpioEncoderCount>::SharedPtr gpioEncoderCountPub;
    rclcpp::Service<hal_pigpio_interfaces::srv::HalPigpioReadGpio>::SharedPtr readGpioService;
    rclcpp::Service<hal_pigpio_interfaces::srv::HalPigpioSetCallback>::SharedPtr setCallbackService;
    rclcpp::Service<hal_pigpio_interfaces::srv::HalPigpioSetEncoderCallback>::SharedPtr setEncoderCallbackService;
    rclcpp::Service<hal_pigpio_interfaces::srv::HalPigpioSetMotorDirection>::SharedPtr setMotorDirectionService;
    std::vector<uint> callbackList;
    std::vector<Motor> motors;
    inline void gpioEdgeChangeCallback(int handle, unsigned gpioId, unsigned edgeChangeType, uint32_t timeSinceBoot_us);
    static void c_gpioEdgeChangeCallback(int handle, unsigned gpioId, unsigned edgeChangeType, uint32_t timeSinceBoot_us, void *userData);
    inline void gpioEncoderEdgeChangeCallback(int handle, unsigned gpioId, unsigned edgeChangeType, uint32_t timeSinceBoot_us);
    static void c_gpioEncoderEdgeChangeCallback(int handle, unsigned gpioId, unsigned edgeChangeType, uint32_t timeSinceBoot_us, void *userData);

public:
    PigpioInput(std::shared_ptr<rclcpp::Node> node, int pigpioHandle);
    ~PigpioInput();
    void readGpio(const std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioReadGpio::Request> request,
                  std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioReadGpio::Response> response);
    void setCallback(const std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioSetCallback::Request> request,
                     std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioSetCallback::Response> response);
    void setEncoderCallback(const std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioSetEncoderCallback::Request> request,
                            std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioSetEncoderCallback::Response> response);
    void setMotorDirection(const std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioSetMotorDirection::Request> request,
                           std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioSetMotorDirection::Response> response);
    void publishEncoderCount(void);
};

#endif