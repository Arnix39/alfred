#include "hal_imuDmpWritingServer.hpp"
#include "hal_imuInterfaces.hpp"

/* Services interface implementation */
ImuClientsRos::ImuClientsRos(ros::NodeHandle *node)
{

    i2cReadByteDataClientRos = node->serviceClient<hal_pigpio::hal_pigpioI2cReadByteData>("hal_pigpioI2cReadByteData");
    i2cWriteByteDataClientRos = node->serviceClient<hal_pigpio::hal_pigpioI2cWriteByteData>("hal_pigpioI2cWriteByteData");
}

ros::ServiceClient ImuClientsRos::getReadByteDataClientHandle()
{
    return i2cReadByteDataClientRos;
}

ros::ServiceClient ImuClientsRos::getWriteByteDataClientHandle()
{
    return i2cWriteByteDataClientRos;
}

/* IMU implementation */
ImuDmpWritingServer::ImuDmpWritingServer(ros::NodeHandle *node) : imuDmpWritingServer(*node, "imuDMPWriting", boost::bind(&ImuDmpWritingServer::writeDmp, this, _1), false),
                                                                  nodeHandle(node)
{
    imuDmpWritingServer.start();
}

void ImuDmpWritingServer::writeDmp(const hal_imu::hal_imuWriteDmpGoalConstPtr &goal)
{
    if (true)
    {
        result.success = true;
        imuDmpWritingServer.setSucceeded(result);
    }
    else
    {
        result.success = false;
        imuDmpWritingServer.setAborted(result);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hal_imuInit");
    ros::NodeHandle node;

    ImuDmpWritingServer imuDmpWritingServer(&node);

    ros::spin();

    return 0;
}