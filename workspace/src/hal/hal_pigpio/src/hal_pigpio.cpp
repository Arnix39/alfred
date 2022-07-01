#include "hal_pigpioInit.hpp"
#include "hal_pigpioInput.hpp"
#include "hal_pigpioOutput.hpp"
#include "hal_pigpioI2c.hpp"
#include "hal_pigpioImu.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv, "hal_pigpio");
    rclcpp::NodeHandle node;
    int pigpioHandle;

    pigpioHandle = pigpio_start(NULL, NULL);
    if (pigpioHandle < 0)
    {
        RCLCPP_ERROR("Pigpio daemon not running!");
        return -1;
    }
    RCLCPP_INFO("Pigpio handle: %d.", pigpioHandle);

    PigpioInit pigpioInit(&node, pigpioHandle);
    PigpioOutput pigpioOutput(&node, pigpioHandle);
    PigpioInput pigpioInput(&node, pigpioHandle);
    PigpioI2c pigpioI2c(&node, pigpioHandle);
    PigpioImu pigpioImu(&node, pigpioHandle);

    rclcpp::Timer heartbeatTimer(node.createTimer(rclcpp::Duration(0.1), &PigpioInit::publishHeartbeat, &pigpioInit));
    rclcpp::Timer encoderCountTimer(node.createTimer(rclcpp::Duration(0.005), &PigpioInput::publishEncoderCount, &pigpioInput));

    rclcpp::spin();

    return 0;
}