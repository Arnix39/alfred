#include "ros/ros.h"
#include "std_msgs/String.h"

void callback(const std_msgs::String::ConstPtr &message)
{
    ROS_INFO("I heard: %s", message->data.c_str());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "app_helloWorld");
    ros::NodeHandle node;

    ros::Subscriber helloWorldsub = node.subscribe("HelloWorld", 1000, callback);

    ros::spin();

    return 0;
}