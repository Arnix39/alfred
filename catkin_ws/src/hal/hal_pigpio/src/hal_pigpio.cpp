#include "hal_pigpioInit.hpp"
#include "hal_pigpioInput.hpp"
#include "hal_pigpioOutput.hpp"
#include "hal_pigpioI2c.hpp"
#include "hal_pigpioImu.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hal_pigpio");
    ros::NodeHandle node;
    int pigpioHandle;

    pigpioHandle = pigpio_start(NULL, NULL);
    if (pigpioHandle < 0)
    {
        ROS_ERROR("Pigpio daemon not running!");
        return -1;
    }
    ROS_INFO("Pigpio handle: %d.", pigpioHandle);

    PigpioInit pigpioInit(&node, pigpioHandle);
    PigpioOutput pigpioOutput(&node, pigpioHandle);
    PigpioInput pigpioInput(&node, pigpioHandle);
    PigpioI2c pigpioI2c(&node, pigpioHandle);
    PigpioImu pigpioImu(&node, pigpioHandle);

    ros::Timer heartbeatTimer(node.createTimer(ros::Duration(0.1), &PigpioInit::publishHeartbeat, &pigpioInit));
    ros::Timer encoderCountTimer(node.createTimer(ros::Duration(0.5), &PigpioInput::publishEncoderCount, &pigpioInput));

    ros::spin();

    return 0;
}