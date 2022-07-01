#ifndef HAL_PIGPIO_INIT
#define HAL_PIGPIO_INIT

#include "rclcpp/rclcpp.hpp"

// Pigpio library
#include <pigpiod_if2.h>

// Services and messages headers (generated)
#include "hal_pigpio/srv/hal_pigpio_get_handle.hpp"
#include "hal_pigpio/srv/hal_pigpio_get_mode.hpp"
#include "hal_pigpio/srv/hal_pigpio_set_input_mode.hpp"
#include "hal_pigpio/srv/hal_pigpio_set_output_mode.hpp"
#include "hal_pigpio/srv/hal_pigpio_set_pull_up.hpp"
#include "hal_pigpio/srv/hal_pigpio_set_pull_down.hpp"
#include "hal_pigpio/srv/hal_pigpio_clear_resistor.hpp"
#include "hal_pigpio/msg/hal_pigpio_heartbeat.hpp"

class PigpioInit
{
private:
    int pigpioHandle;
    std::shared_ptr<rclcpp::Node> halPigpioNode;
    rclcpp::ServiceServer getHandleService;
    rclcpp::ServiceServer getModeService;
    rclcpp::ServiceServer setInputModeService;
    rclcpp::ServiceServer setOutputModeService;
    rclcpp::ServiceServer setPullUpService;
    rclcpp::ServiceServer setPullDownService;
    rclcpp::ServiceServer clearResistorService;
    rclcpp::Publisher heartbeatPublisher;

public:
    PigpioInit(std::shared_ptr<rclcpp::Node> node, int pigpioHandle);
    ~PigpioInit();
    bool getHandle(hal_pigpio::hal_pigpioGetHandle::Request &req,
                   hal_pigpio::hal_pigpioGetHandle::Response &res);
    bool getMode(hal_pigpio::hal_pigpioGetMode::Request &req,
                 hal_pigpio::hal_pigpioGetMode::Response &res);
    bool setInputMode(hal_pigpio::hal_pigpioSetInputMode::Request &req,
                      hal_pigpio::hal_pigpioSetInputMode::Response &res);
    bool setOutputMode(hal_pigpio::hal_pigpioSetOutputMode::Request &req,
                       hal_pigpio::hal_pigpioSetOutputMode::Response &res);
    bool setPullUp(hal_pigpio::hal_pigpioSetPullUp::Request &req,
                   hal_pigpio::hal_pigpioSetPullUp::Response &res);
    bool setPullDown(hal_pigpio::hal_pigpioSetPullDown::Request &req,
                     hal_pigpio::hal_pigpioSetPullDown::Response &res);
    bool clearResistor(hal_pigpio::hal_pigpioClearResistor::Request &req,
                       hal_pigpio::hal_pigpioClearResistor::Response &res);
    void publishHeartbeat(const rclcpp::TimerEvent &timerEvent);
};

#endif