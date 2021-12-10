#include "ros/ros.h"
#include "std_msgs/String.h"
#include <pigpiod_if2.h>

int main(int argc, char **argv)
{
    int pigpio_handle;
    ros::init(argc, argv, "hal_helloWorld");
    ros::NodeHandle node;

    ros::Publisher helloWorldPub = node.advertise<std_msgs::String>("HelloWorld", 1000);

    ros::Rate loop_rate(1);
    pigpio_handle = pigpio_start(NULL, NULL);
    set_PWM_frequency(pigpio_handle, 14, 1000);
    set_PWM_dutycycle(pigpio_handle, 14, 128);

    int count = 0;
    while (ros::ok())
    {
        std_msgs::String helloWorldMessage;
        std::stringstream stream;
        stream << "Hello world " << count;
        helloWorldMessage.data = stream.str();

        helloWorldPub.publish(helloWorldMessage);

        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }

    pigpio_stop(pigpio_handle);
    return 0;
}