#ifndef HAL_PROXSENS
#define HAL_PROXSENS

#include "stdint.h"

#include "rclcpp/rclcpp.hpp"

// Services and messages headers (generated)
#include "hal_pigpio_interfaces/srv/hal_pigpio_set_input_mode.hpp"
#include "hal_pigpio_interfaces/srv/hal_pigpio_set_output_mode.hpp"
#include "hal_pigpio_interfaces/srv/hal_pigpio_set_gpio_high.hpp"
#include "hal_pigpio_interfaces/srv/hal_pigpio_send_trigger_pulse.hpp"
#include "hal_pigpio_interfaces/srv/hal_pigpio_set_callback.hpp"
#include "hal_pigpio_interfaces/msg/hal_pigpio_edge_change.hpp"
#include "hal_pigpio_interfaces/msg/hal_pigpio_heartbeat.hpp"
#include "hal_proxsens_interfaces/msg/hal_proxsens.hpp"

#define PROXSENS_TRIGGER_GPIO 5
#define PROXSENS_ECHO_GPIO 6
#define PROXSENS_LEVEL_SHIFTER_OE_GPIO 10
#define PROXSENS_TRIGGER_LENGTH_US 20

#define AS_RISING_EDGE 0
#define AS_FALLING_EDGE 1
#define AS_EITHER_EDGE 2

#define FALLING_EDGE 0
#define RISING_EDGE 1
#define NO_CHANGE 2

class Proxsens
{
private:
    std::shared_ptr<rclcpp::Node> halProxsensNode;
    uint8_t edgeChangeType;
    uint32_t timestamp;
    uint32_t echoCallbackId;
    uint16_t distanceInCm;
    bool pigpioNodeStarted;
    bool isStarted;
    rclcpp::Client<hal_pigpio_interfaces::srv::HalPigpioSetInputMode>::SharedPtr gpioSetInputClient;
    rclcpp::Client<hal_pigpio_interfaces::srv::HalPigpioSetOutputMode>::SharedPtr gpioSetOutputClient;
    rclcpp::Client<hal_pigpio_interfaces::srv::HalPigpioSetCallback>::SharedPtr gpioSetCallbackClient;
    rclcpp::Client<hal_pigpio_interfaces::srv::HalPigpioSendTriggerPulse>::SharedPtr gpioSendTriggerPulseClient;
    rclcpp::Client<hal_pigpio_interfaces::srv::HalPigpioSetGpioHigh>::SharedPtr gpioSetGpioHighClient;
    rclcpp::Publisher<hal_proxsens_interfaces::msg::HalProxsens>::SharedPtr proxsensDistancePub;
    rclcpp::Subscription<hal_pigpio_interfaces::msg::HalPigpioEdgeChange>::SharedPtr proxsensEdgeChangeSub;
    rclcpp::Subscription<hal_pigpio_interfaces::msg::HalPigpioHeartbeat>::SharedPtr proxsensPigpioHBSub;

public:
    Proxsens(std::shared_ptr<rclcpp::Node> node);
    ~Proxsens() = default;
    void publishDistance(void);
    void configureGpios(void);
    void trigger(void);
    void enableOutputLevelShifter(void);
    void edgeChangeCallback(const hal_pigpio_interfaces::msg::HalPigpioEdgeChange &msg);
    void pigpioHeartbeatCallback(const hal_pigpio_interfaces::msg::HalPigpioHeartbeat &msg);
    bool isPigpioNodeStarted(void);
    void publishAndGetDistance(void);
    bool isNotStarted(void);
    void starts(void);
};

#endif