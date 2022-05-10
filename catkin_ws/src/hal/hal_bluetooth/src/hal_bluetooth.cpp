#include "hal_bluetooth.hpp"
#include "hal_bluetoothInterfaces.hpp"

/* Publisher interface implementation */
BluetoothPublisherRos::BluetoothPublisherRos(ros::NodeHandle *node) : bluetoothPubRos(node->advertise<hal_bluetooth::hal_bluetoothMsg>("bluetoothCommand", 1000))
{
}

void BluetoothPublisherRos::publish(hal_bluetooth::hal_bluetoothMsg message)
{
    bluetoothPubRos.publish(message);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hal_bluetooth");
    ros::NodeHandle node;

    BluetoothPublisherRos bluetoothPublisherRos(&node);

    ROS_INFO("bluetooth node starting...");
    while (ros::ok())
    {
        ros::spinOnce();
    }

    return 0;
}