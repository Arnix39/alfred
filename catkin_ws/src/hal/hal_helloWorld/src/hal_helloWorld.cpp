#include "ros/ros.h"
#include "std_msgs/String.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hal_helloWorld");
    ros::NodeHandle node;

    ros::Publisher helloWorldPub = node.advertise<std_msgs::String>("HelloWorld", 1000);

    ros::Rate loop_rate(1);

    int count = 0;
    while (ros::ok())
    {
        std_msgs::String helloWorldMessage;
        std::stringstream stream;
        stream << "Hello world " << count;
        helloWorldMessage.data = stream.str();

        helloWorldPub.publish(helloWorldMessage);

        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }

    return 0;
}