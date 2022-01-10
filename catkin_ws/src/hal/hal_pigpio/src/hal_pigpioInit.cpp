#include "pigpioInit.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hal_pigpioInit");
    ros::NodeHandle node;

    ros::spin();

    return 0;
}