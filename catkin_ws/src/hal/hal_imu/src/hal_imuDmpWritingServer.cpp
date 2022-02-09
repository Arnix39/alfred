#include "hal_imuDmpWritingServer.hpp"

ImuDmpWritingServer::ImuDmpWritingServer(ros::NodeHandle *node) : imuDmpWritingServer(&node, "imuDMPWriting", boost::bind(&ImuDmpWritingServer::writeDmp, this, _1), false),
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