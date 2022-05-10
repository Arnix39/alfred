#ifndef HAL_BLUETOOTH_INTERFACES
#define HAL_BLUETOOTH_INTERFACES

#include "hal_bluetoothVirtuals.hpp"

class BluetoothPublisherRos : public BluetoothPublisher
{
private:
    ros::Publisher bluetoothPubRos;

public:
    BluetoothPublisherRos(ros::NodeHandle *node);
    ~BluetoothPublisherRos() = default;
    void publish(hal_bluetooth::hal_bluetoothMsg message) override;
};

#endif