#include "hal_imu.hpp"
#include "hal_imuInterfaces.hpp"

/* Publisher interface implementation */
ImuPublisherRos::ImuPublisherRos(ros::NodeHandle *node) : imuPublisherRos(node->advertise<hal_imu::hal_imuMsg>("angleValue", 1000))
{
}

void ImuPublisherRos::publish(hal_imu::hal_imuMsg message)
{
    imuPublisherRos.publish(message);
}

/* Services servers interface implementation */
ImuServersRos::ImuServersRos(ros::NodeHandle *node) : nodeHandle(node)
{
}

void ImuServersRos::advertiseGetHandleService(Imu *imu)
{
    imuGetHandleServerRos = nodeHandle->advertiseService("hal_imuGetHandle", &Imu::getHandle, imu);
}

/* Services clients interface implementation */
ImuClientsRos::ImuClientsRos(ros::NodeHandle *node) : i2cOpenClientRos(node->serviceClient<hal_pigpio::hal_pigpioI2cOpen>("hal_pigpioI2cOpen")),
                                                      i2cCloseClientRos(node->serviceClient<hal_pigpio::hal_pigpioI2cClose>("hal_pigpioI2cClose"))
{
}

ros::ServiceClient *ImuClientsRos::getI2cOpenHandle()
{
    return &i2cOpenClientRos;
}

ros::ServiceClient *ImuClientsRos::getI2cCloseHandle()
{
    return &i2cCloseClientRos;
}

/* IMU implementation */
Imu::Imu(ImuPublisher *imuMessagePublisher, ImuServers *imuServiceServers, ImuClients *imuServiceClients) : imuPublisher(imuMessagePublisher),
                                                                                                            imuServers(imuServiceServers),
                                                                                                            imuClients(imuServiceClients)
{
    imuServers->advertiseGetHandleService(this);
}

bool Imu::getHandle(hal_imu::hal_imuGetHandle::Request &req,
                    hal_imu::hal_imuGetHandle::Response &res)
{
    res.handle = imuHandle;
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hal_imu");
    ros::NodeHandle node;

    ImuPublisherRos imuMessagePublisherRos(&node);
    ImuServersRos imuServiceServersRos(&node);
    ImuClientsRos imuServiceClientsRos(&node);

    Imu Imu(&imuMessagePublisherRos, &imuServiceServersRos, &imuServiceClientsRos);

    ros::spin();

    return 0;
}