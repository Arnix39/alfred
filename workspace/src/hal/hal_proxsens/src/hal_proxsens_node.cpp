#include "hal_proxsens.hpp"

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("hal_proxsens_node");

    Proxsens proxSens(node);

    RCLCPP_INFO(node->get_logger(),"proxSens node waiting for pigpio node to start...");
    while (rclcpp::ok())
    {
        if (proxSens.isNotStarted() && proxSens.isPigpioNodeStarted())
        {
            RCLCPP_INFO(node->get_logger(),"proxSens node initialising...");
            proxSens.configureGpios();
            proxSens.enableOutputLevelShifter();
            proxSens.starts();
            rclcpp::TimerBase::SharedPtr proxsensTimer(node->create_wall_timer(100ms, std::bind(&Proxsens::publishAndGetDistance, &proxSens)));
            RCLCPP_INFO(node->get_logger(),"proxSens node initialised.");
        }

        rclcpp::spin_some(node);
    }

    rclcpp::shutdown();
    return 0;
}