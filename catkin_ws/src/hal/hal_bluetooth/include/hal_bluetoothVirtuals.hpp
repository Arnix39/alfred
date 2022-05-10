#ifndef HAL_BLUETOOTH_VIRTUALS
#define HAL_BLUETOOTH_VIRTUALS

#include "ros/ros.h"

// Services and messages headers (generated)
#include "hal_bluetooth/hal_bluetoothMsg.h"

class BluetoothPublisher
{
public:
    BluetoothPublisher() {}
    virtual ~BluetoothPublisher() {}
    virtual void publish(hal_bluetooth::hal_bluetoothMsg message) = 0;
};

#endif