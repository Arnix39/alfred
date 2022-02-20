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

Imu::~Imu()
{
    hal_pigpio::hal_pigpioI2cClose i2cCloseSrv;

    i2cCloseSrv.request.handle = imuHandle;

    imuClients->getI2cCloseHandle()->call(i2cCloseSrv);
}

bool Imu::getHandle(hal_imu::hal_imuGetHandle::Request &req,
                    hal_imu::hal_imuGetHandle::Response &res)
{
    res.handle = imuHandle;
    return true;
}

void Imu::initI2c(void)
{
    hal_pigpio::hal_pigpioI2cOpen i2cOpenSrv;

    i2cOpenSrv.request.bus = IMU_I2C_BUS;
    i2cOpenSrv.request.address = IMU_I2C_ADDRESS;

    imuClients->getI2cOpenHandle()->call(i2cOpenSrv);
    if (i2cOpenSrv.response.hasSucceeded)
    {
        imuHandle = i2cOpenSrv.response.handle;
    }
    else
    {
        ROS_ERROR("Unable to open I2C communication with device %u on bus %u!", i2cOpenSrv.request.address, i2cOpenSrv.request.bus);
    }
}

void Imu::writeDmp(void)
{
    hal_imu::hal_imuWriteDmpGoal imuDmpWritingGoal;
    imuActionClient_t imuDmpWritingClient("imuDMPWriting", false);
    imuDmpWritingClient.waitForServer();

    imuDmpWritingGoal.write = true;
    imuDmpWritingClient.sendGoal(imuDmpWritingGoal);

    imuDmpWritingClient.waitForResult();
    if (imuDmpWritingClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("DMP code written successfully!");
    }
    else
    {
        ROS_ERROR("Error while writing DMP code!");
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hal_imu");
    ros::NodeHandle node;

    ImuPublisherRos imuMessagePublisherRos(&node);
    ImuServersRos imuServiceServersRos(&node);
    ImuClientsRos imuServiceClientsRos(&node);

    Imu Imu(&imuMessagePublisherRos, &imuServiceServersRos, &imuServiceClientsRos);
    Imu.initI2c();
    Imu.writeDmp();

    ros::Rate loop_rate(200);

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}