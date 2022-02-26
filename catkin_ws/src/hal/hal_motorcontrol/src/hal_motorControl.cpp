#include "hal_motorControl.hpp"
#include "hal_motorControlInterfaces.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hal_motorcontrol");
    ros::NodeHandle node;

    ros::Rate loop_rate(100);

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}