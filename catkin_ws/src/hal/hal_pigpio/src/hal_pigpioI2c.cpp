#include "pigpioI2c.hpp"

// Services headers (generated)
#include "hal_pigpio/hal_pigpioGetHandle.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hal_pigpioI2c");
    ros::NodeHandle node;
    int pigpio_handle;

    ros::ServiceClient getPigpioHandle = node.serviceClient<hal_pigpio::hal_pigpioGetHandle>("hal_pigpioGetHandle");
    hal_pigpio::hal_pigpioGetHandle pigpioHandleRequest;
    getPigpioHandle.call(pigpioHandleRequest);
    pigpio_handle = pigpioHandleRequest.response.handle;

    PigpioI2c pigpioI2c = PigpioI2c(&node, pigpio_handle);

    ros::spin();

    return 0;
}