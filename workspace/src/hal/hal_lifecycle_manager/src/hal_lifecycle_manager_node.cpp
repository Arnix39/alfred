#include "hal_lifecycle_manager.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HalLifecycleManager>();

    node->ActivateNodes();

    rclcpp::spin(node);

    node->DeactivateNodes();
    node->ShutdownNodes();

    rclcpp::shutdown();
    return 0;
}