#include "hal_proxsens.hpp"

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Proxsens>();

    RCLCPP_INFO(node->get_logger(),"proxSens node waiting for pigpio node to start...");
    while (rclcpp::ok())
    {
        if (node->isNotStarted() && node->isPigpioNodeStarted())
        {
            RCLCPP_INFO(node->get_logger(),"proxSens node initialising...");
            node->configureGpios();
            node->enableOutputLevelShifter();
            node->starts();
            RCLCPP_INFO(node->get_logger(),"proxSens node initialised.");
        }

        rclcpp::spin_some(node);
    }

    rclcpp::shutdown();
    return 0;
}