#include "app_controller.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "app_controller");
    ros::NodeHandle node;

    ROS_INFO("controller node starting...");
    while (ros::ok())
    {
        ros::spinOnce();
    }

    return 0;
}