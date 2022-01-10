#include "pigpioOutput.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hal_pigpioOutput");
    ros::NodeHandle node;
    int pigpio_handle;

    ros::ServiceClient getPigpioHandle = node.serviceClient<hal_pigpio::hal_pigpioGetHandle>("hal_pigpioGetHandle");
    hal_pigpio::hal_pigpioGetHandle pigpioHandleRequest;
    getPigpioHandle.call(pigpioHandleRequest);
    pigpio_handle = pigpioHandleRequest.response.handle;

    PigpioOutput pigpioOutput = PigpioOutput(&node, pigpio_handle);

    ros::spin();

    return 0;
}