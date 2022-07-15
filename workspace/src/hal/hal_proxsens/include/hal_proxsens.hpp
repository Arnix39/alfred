#ifndef HAL_PROXSENS
#define HAL_PROXSENS

#include "stdint.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

// Services and messages headers (generated)
#include "hal_pigpio_interfaces/srv/hal_pigpio_set_input_mode.hpp"
#include "hal_pigpio_interfaces/srv/hal_pigpio_set_output_mode.hpp"
#include "hal_pigpio_interfaces/srv/hal_pigpio_set_gpio_high.hpp"
#include "hal_pigpio_interfaces/srv/hal_pigpio_send_trigger_pulse.hpp"
#include "hal_pigpio_interfaces/srv/hal_pigpio_set_callback.hpp"
#include "hal_pigpio_interfaces/msg/hal_pigpio_edge_change.hpp"
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

class Proxsens : public rclcpp_lifecycle::LifecycleNode
{
private:
    uint8_t edgeChangeType;
    uint32_t timestamp;
    uint32_t echoCallbackId;
    uint16_t distanceInCm;
    rclcpp::Client<hal_pigpio_interfaces::srv::HalPigpioSetInputMode>::SharedPtr gpioSetInputClient;
    rclcpp::Client<hal_pigpio_interfaces::srv::HalPigpioSetOutputMode>::SharedPtr gpioSetOutputClient;
    rclcpp::Client<hal_pigpio_interfaces::srv::HalPigpioSetCallback>::SharedPtr gpioSetCallbackClient;
    rclcpp::Client<hal_pigpio_interfaces::srv::HalPigpioSendTriggerPulse>::SharedPtr gpioSendTriggerPulseClient;
    rclcpp::Client<hal_pigpio_interfaces::srv::HalPigpioSetGpioHigh>::SharedPtr gpioSetGpioHighClient;
    rclcpp_lifecycle::LifecyclePublisher<hal_proxsens_interfaces::msg::HalProxsens>::SharedPtr proxsensDistancePub;
    rclcpp::Subscription<hal_pigpio_interfaces::msg::HalPigpioEdgeChange>::SharedPtr proxsensEdgeChangeSub;

public:
    Proxsens();
    ~Proxsens() = default;
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state);
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state);
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state);
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state);
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state);
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state);
    void publishDistance(void);
    void configureGpios(void);
    void trigger(void);
    void enableOutputLevelShifter(void);
    void edgeChangeCallback(const hal_pigpio_interfaces::msg::HalPigpioEdgeChange &msg);
    void publishAndGetDistance(void);
};

#endif