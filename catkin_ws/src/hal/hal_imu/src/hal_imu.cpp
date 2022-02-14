#include "hal_imu.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hal_imu");
    ros::NodeHandle node;

    ros::spin();

    return 0;
}