#include "hal_pigpioInit.hpp"
#include "hal_pigpioInput.hpp"
#include "hal_pigpioOutput.hpp"
#include "hal_pigpioI2c.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hal_pigpioInit");
    ros::NodeHandle node;

    PigpioInit pigpioInit(&node);
    PigpioOutput pigpioOutput(&node);
    PigpioInput pigpioInput(&node);
    PigpioI2c pigpioI2c(&node);

    ros::spin();

    return 0;
}