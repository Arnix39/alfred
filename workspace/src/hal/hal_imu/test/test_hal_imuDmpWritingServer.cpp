// Copyright (c) 2022 Arnix Robotix
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "gtest/gtest.h"
#include "gmock/gmock.h"

#include "hal_imuDmpWritingServer.hpp"

/* Action server interface mock */
class ImuActionServerMock : public ImuActionServer
{
private:
  imuActionServer_t * imuWriteDmpServer;

public:
  explicit ImuActionServerMock(imuActionServer_t * imuWriteDmpServerMock);
  ~ImuActionServerMock() = default;
  void registerCallback(ImuDmpWritingServer * imuDmpWritingServer) override;
  imuActionServer_t * getActionServerHandle() override;
};

ImuActionServerMock::ImuActionServerMock(imuActionServer_t * imuWriteDmpServerMock)
: imuWriteDmpServer(imuWriteDmpServerMock)
{
}

void ImuActionServerMock::registerCallback(ImuDmpWritingServer * imuDmpWritingServer)
{
  (void)imuDmpWritingServer;
}

imuActionServer_t * ImuActionServerMock::getActionServerHandle()
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
  ros::ServiceClient * getReadByteDataClientHandle() override;
  ros::ServiceClient * getWriteByteDataClientHandle() override;
};

ros::ServiceClient * ImuClientsMock::getReadByteDataClientHandle()
{
  return &i2cReadByteDataClientRos;
}

ros::ServiceClient * ImuClientsMock::getWriteByteDataClientHandle()
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
  ImuDmpWritingServerTest()
  : imuWriteDmpServerMock(node, "imuDMPWriting", false),
    imuWriteDmpServer(&imuWriteDmpServerMock),
    imuDmpWritingServer(&imuWriteDmpServer, &imuServiceClients)
  {
  }
};

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_hal_imu");
  return RUN_ALL_TESTS();
}
