#include "gtest/gtest.h"
#include "gmock/gmock.h"

#include "hal_imuDmpWritingServer.hpp"

/* Action server interface mock */
class ImuActionServerMock : public ImuActionServer
{
private:
    imuActionServer_t *imuWriteDmpServer;

public:
    ImuActionServerMock(imuActionServer_t *imuWriteDmpServerMock);
    ~ImuActionServerMock() = default;
    void registerCallback(ImuDmpWritingServer *imuDmpWritingServer) override;
    imuActionServer_t *getActionServerHandle() override;
};

ImuActionServerMock::ImuActionServerMock(imuActionServer_t *imuWriteDmpServerMock) : imuWriteDmpServer(imuWriteDmpServerMock)
{
}

void ImuActionServerMock::registerCallback(ImuDmpWritingServer *imuDmpWritingServer)
{
    (void)imuDmpWritingServer;
}

imuActionServer_t *ImuActionServerMock::getActionServerHandle()
{
    return imuWriteDmpServer;
}

/* Services interface mock */
class ImuClientsMock : public ImuClients
{
private:
    ros::ServiceClient i2cReadByteDataClientRos;
    ros::ServiceClient i2cWriteByteDataClientRos;

public:
    ImuClientsMock() = default;
    ~ImuClientsMock() = default;
    ros::ServiceClient *getReadByteDataClientHandle() override;
    ros::ServiceClient *getWriteByteDataClientHandle() override;
};

ros::ServiceClient *ImuClientsMock::getReadByteDataClientHandle()
{
    return &i2cReadByteDataClientRos;
}

ros::ServiceClient *ImuClientsMock::getWriteByteDataClientHandle()
{
    return &i2cWriteByteDataClientRos;
}

/* Test fixtures */
class ImuDmpWritingServerTest : public testing::Test
{
protected:
    ros::NodeHandle node;
    imuActionServer_t imuWriteDmpServerMock;
    ImuActionServerMock imuWriteDmpServer;
    ImuClientsMock imuServiceClients;
    ImuDmpWritingServer imuDmpWritingServer;

public:
    ImuDmpWritingServerTest() : imuWriteDmpServerMock(node, "imuDMPWriting", false),
                                imuWriteDmpServer(&imuWriteDmpServerMock),
                                imuDmpWritingServer(&imuWriteDmpServer, &imuServiceClients)
    {
    }
};

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "test_hal_imu");
    return RUN_ALL_TESTS();
}