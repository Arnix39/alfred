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

public:
    HalLifecycleManager();
};

#endif