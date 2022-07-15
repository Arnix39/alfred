#include "app_lifecycle_manager.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::shutdown();
    return 0;
}