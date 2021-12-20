#include "ros/ros.h"
#include "std_msgs/String.h"
#include <pigpiod_if2.h>
#include "hal/hal_pigpioGetHandle.h"

static int pigpio_handle;

bool getHandle(hal::hal_pigpioGetHandle::Request &req,
               hal::hal_pigpioGetHandle::Response &res)
{
    res.handle = pigpio_handle;
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hal_pigpioInit");
    ros::NodeHandle node;

    ros::Rate loop_rate(10);
    pigpio_handle = pigpio_start(NULL, NULL);

    while (ros::ok())
    {
        ros::ServiceServer service = node.advertiseService("hal_pigpioGetHandle", getHandle);

        ros::spinOnce();
        loop_rate.sleep();
    }

    pigpio_stop(pigpio_handle);
    return 0;
}