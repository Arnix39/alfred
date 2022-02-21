#include "hal_pigpioInit.hpp"
#include "hal_pigpioInput.hpp"
#include "hal_pigpioOutput.hpp"
#include "hal_pigpioI2c.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hal_pigpio");
    ros::NodeHandle node;
    int pigpioHandle;

    pigpioHandle = pigpio_start(NULL, NULL);
    if (pigpioHandle < 0)
    {
        ROS_ERROR("Failed to start pigpio daemon!");
    }
    ROS_INFO("Pigpio handle: %d.", pigpioHandle);

    PigpioInit pigpioInit(&node, pigpioHandle);
    PigpioOutput pigpioOutput(&node, pigpioHandle);
    PigpioInput pigpioInput(&node, pigpioHandle);
    PigpioI2c pigpioI2c(&node, pigpioHandle);

    ros::spin();

    return 0;
}