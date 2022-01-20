#include "hal_proxSens.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hal_proxsens");
    ros::NodeHandle node;

    ros::Publisher proxSensPub = node.advertise<std_msgs::String>("proxSensorValues", 1000);

    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        std_msgs::String proxSensMessage;
        
        std::stringstream stream;
        stream << "Proximity sensor values: NaN";
        proxSensMessage.data = stream.str();

        proxSensPub.publish(proxSensMessage);

        ros::spinOnce();
        loop_rate.sleep();
    }
        
    return 0;
}