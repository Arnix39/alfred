#include "hal_pigpioInit.hpp"
#include "hal_pigpioInput.hpp"
#include "hal_pigpioOutput.hpp"
#include "hal_pigpioI2c.hpp"
#include "hal_pigpioImu.hpp"

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("hal_pigpio_node");
    int pigpioHandle;

    pigpioHandle = pigpio_start(NULL, NULL);
    if (pigpioHandle < 0)
    {
        RCLCPP_ERROR(node->get_logger(),"Pigpio daemon not running!");
        return -1;
    }
    RCLCPP_INFO(node->get_logger(),"Pigpio handle: %d.", pigpioHandle);

    PigpioInit pigpioInit(node, pigpioHandle);
    PigpioOutput pigpioOutput(node, pigpioHandle);
    PigpioInput pigpioInput(node, pigpioHandle);
    PigpioI2c pigpioI2c(node, pigpioHandle);
    PigpioImu pigpioImu(node, pigpioHandle);

    rclcpp::TimerBase::SharedPtr heartbeatTimer(node->create_wall_timer(100ms, std::bind(&PigpioInit::publishHeartbeat, &pigpioInit)));
    rclcpp::TimerBase::SharedPtr encoderCountTimer(node->create_wall_timer(5ms, std::bind(&PigpioInput::publishEncoderCount, &pigpioInput)));

    rclcpp::spin(node);

    return 0;
}