#ifndef HAL_PIGPIO_INIT
#define HAL_PIGPIO_INIT

#include "rclcpp/rclcpp.hpp"

// Pigpio library
#include <pigpiod_if2.h>

// Services and messages headers (generated)
#include "hal_pigpio_interfaces/srv/hal_pigpio_get_handle.hpp"
#include "hal_pigpio_interfaces/srv/hal_pigpio_get_mode.hpp"
#include "hal_pigpio_interfaces/srv/hal_pigpio_set_input_mode.hpp"
#include "hal_pigpio_interfaces/srv/hal_pigpio_set_output_mode.hpp"
#include "hal_pigpio_interfaces/srv/hal_pigpio_set_pull_up.hpp"
#include "hal_pigpio_interfaces/srv/hal_pigpio_set_pull_down.hpp"
#include "hal_pigpio_interfaces/srv/hal_pigpio_clear_resistor.hpp"
#include "hal_pigpio_interfaces/msg/hal_pigpio_heartbeat.hpp"

class PigpioInit
{
private:
    int pigpioHandle;
    std::shared_ptr<rclcpp::Node> halPigpioNode;
    rclcpp::Service<hal_pigpio_interfaces::srv::HalPigpioGetHandle>::SharedPtr getHandleService;
    rclcpp::Service<hal_pigpio_interfaces::srv::HalPigpioGetMode>::SharedPtr getModeService;
    rclcpp::Service<hal_pigpio_interfaces::srv::HalPigpioSetInputMode>::SharedPtr setInputModeService;
    rclcpp::Service<hal_pigpio_interfaces::srv::HalPigpioSetOutputMode>::SharedPtr setOutputModeService;
    rclcpp::Service<hal_pigpio_interfaces::srv::HalPigpioSetPullUp>::SharedPtr setPullUpService;
    rclcpp::Service<hal_pigpio_interfaces::srv::HalPigpioSetPullDown>::SharedPtr setPullDownService;
    rclcpp::Service<hal_pigpio_interfaces::srv::HalPigpioClearResistor>::SharedPtr clearResistorService;
    rclcpp::Publisher<hal_pigpio_interfaces::msg::HalPigpioHeartbeat>::SharedPtr heartbeatPublisher;

public:
    PigpioInit(std::shared_ptr<rclcpp::Node> node, int pigpioHandle);
    ~PigpioInit();
    void getHandle(const std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioGetHandle::Request> request,
                   std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioGetHandle::Response> response);
    void getMode(const std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioGetMode::Request> request,
                 std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioGetMode::Response> response);
    void setInputMode(const std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioSetInputMode::Request> request,
                      std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioSetInputMode::Response> response);
    void setOutputMode(const std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioSetOutputMode::Request> request,
                       std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioSetOutputMode::Response> response);
    void setPullUp(const std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioSetPullUp::Request> request,
                   std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioSetPullUp::Response> response);
    void setPullDown(const std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioSetPullDown::Request> request,
                     std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioSetPullDown::Response> response);
    void clearResistor(const std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioClearResistor::Request> request,
                       std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioClearResistor::Response> response);
    void publishHeartbeat(void);
};

#endif