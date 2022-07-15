#ifndef HAL_LIFECYCLE_MANAGER
#define HAL_LIFECYCLE_MANAGER

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"

class HalLifecycleManager : public rclcpp::Node
{
private:
    std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::GetState>> hal_pigpio_node_get_state;
    std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::GetState>> hal_proxsens_node_get_state;
    std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::ChangeState>> hal_pigpio_node_change_state;
    std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::ChangeState>> hal_proxsens_node_change_state;

public:
    HalLifecycleManager();
};

#endif