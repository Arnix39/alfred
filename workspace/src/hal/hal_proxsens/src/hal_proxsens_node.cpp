#include "hal_proxsens.hpp"

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Proxsens>();

    rclcpp::spin(node->get_node_base_interface());

    rclcpp::shutdown();
    return 0;
}