#include "ros/ros.h"
#include "std_msgs/String.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hal_imu");
    ros::NodeHandle node;

    ros::Publisher imuPub = node.advertise<std_msgs::String>("IMUValues", 1000);

    std_msgs::String imuMessage;
    std::stringstream stream;
    stream << "IMU values: NaN";
    imuMessage.data = stream.str();

    imuPub.publish(imuMessage);

    ros::spin();

    return 0;
}