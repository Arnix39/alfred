#include "hal_proxSens.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hal_proxsens");
    ros::NodeHandle node;

    ros::Publisher proxSensPub = node.advertise<hal_proxsens::hal_proxsensMsg>("proxSensorValue", 1000);

    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        hal_proxsens::hal_proxsensMsg proxSensMessage; 
        proxSensMessage.distanceInCm = 100;

        proxSensPub.publish(proxSensMessage);

        ros::spinOnce();
        loop_rate.sleep();
    }
        
    return 0;
}