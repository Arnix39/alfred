#include "hal_pigpio.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);    
    auto node = std::make_shared<Pigpio>();

    rclcpp::spin(node->get_node_base_interface());
    
    rclcpp::shutdown();
    return 0;
}