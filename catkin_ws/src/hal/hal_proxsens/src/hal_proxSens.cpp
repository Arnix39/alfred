#include "ros/ros.h"
#include "std_msgs/String.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hal_proxsens");
    ros::NodeHandle node;

    ros::Publisher proxSensPub = node.advertise<std_msgs::String>("proxSensorValues", 1000);

    std_msgs::String proxSensMessage;
    std::stringstream stream;
    stream << "Proximity sensor values: NaN";
    proxSensMessage.data = stream.str();

    proxSensPub.publish(proxSensMessage);

    ros::spin();

    return 0;
}