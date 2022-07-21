#include "hal_imuDmpWritingServer.hpp"
#include "hal_imuDmpWritingServerInterfaces.hpp"
#include "hal_mpu6050.hpp"

/* Action server interface implementation */
ImuDmpWritingActionServerRos::ImuDmpWritingActionServerRos(ros::NodeHandle *node) : imuWriteDmpServerRos(*node, "imuDMPWriting", false)
{
}

void ImuDmpWritingActionServerRos::registerCallback(ImuDmpWritingServer *imuDmpWritingServer)
{
    imuWriteDmpServerRos.registerGoalCallback((std::function<void()>)std::bind(&ImuDmpWritingServer::writeDmp, imuDmpWritingServer));
}

imuDmpWritingActionServer_t *ImuDmpWritingActionServerRos::getActionServerHandle()
{
    return &imuWriteDmpServerRos;
}

/* Services interface implementation */
ImuDmpWritingClientsRos::ImuDmpWritingClientsRos(ros::NodeHandle *node) : i2cWriteByteDataClientRos(node->serviceClient<hal_pigpio::hal_pigpioI2cWriteByteData>("hal_pigpioI2cWriteByteData")),
                                                                          i2cWriteBlockDataClientRos(node->serviceClient<hal_pigpio::hal_pigpioI2cWriteBlockData>("hal_pigpioI2cWriteBlockData")),
                                                                          i2cGetHandleClientRos(node->serviceClient<hal_imu::hal_imuGetHandle>("hal_imuGetHandle"))
{
}

ros::ServiceClient *ImuDmpWritingClientsRos::getWriteByteDataClientHandle()
{
    return &i2cWriteByteDataClientRos;
}

ros::ServiceClient *ImuDmpWritingClientsRos::getWriteBlockDataClientHandle()
{
    return &i2cWriteBlockDataClientRos;
}

ros::ServiceClient *ImuDmpWritingClientsRos::getGetHandleClientHandle()
{
    return &i2cGetHandleClientRos;
}

/* Subscriber interface implementation */
ImuDmpWritingServerSubscribersRos::ImuDmpWritingServerSubscribersRos(ros::NodeHandle *node) : nodeHandle(node)
{
}

void ImuDmpWritingServerSubscribersRos::subscribe(ImuDmpWritingServer *imuDmpWritingServer)
{
    imuDmpWritingServerImuI2cInitHBSubRos = nodeHandle->subscribe("hal_imuI2cHeartbeatMsg", 1000, &ImuDmpWritingServer::imuDmpWritingServerI2cInitHeartbeatCallback, imuDmpWritingServer);
}

/* IMU DMP implementation */
ImuDmpWritingServer::ImuDmpWritingServer(ImuDmpWritingActionServer *imuWriteDmpServer, ImuDmpWritingClients *imuDmpServiceClients, ImuDmpWritingServerSubscribers *imuDmpWritingServerSubscribers) : imuDmpWritingServer(imuWriteDmpServer),
                                                                                                                                                                                                     imuDmpClients(imuDmpServiceClients),
                                                                                                                                                                                                     imuDmpWritingServerSubs(imuDmpWritingServerSubscribers),
                                                                                                                                                                                                     i2cInitialised(false),
                                                                                                                                                                                                     isStarted(false),
                                                                                                                                                                                                     imuHandle(-1)
{
    imuDmpWritingServerSubs->subscribe(this);

    imuDmpWritingServer->registerCallback(this);
}

void ImuDmpWritingServer::getI2cHandle(void)
{
    hal_imu::hal_imuGetHandle i2cGetHandleSrv;
    imuDmpClients->getGetHandleClientHandle()->call(i2cGetHandleSrv);
    imuHandle = i2cGetHandleSrv.response.handle;
}

void ImuDmpWritingServer::startServer(void)
{
    imuDmpWritingServer->getActionServerHandle()->start();
    RCLCPP_INFO(get_logger(), "Action server started.");
}

bool ImuDmpWritingServer::isI2cInitialised(void)
{
    return i2cInitialised;
}

void ImuDmpWritingServer::imuDmpWritingServerI2cInitHeartbeatCallback(const hal_imu::hal_imuI2cHeartbeatMsg &msg)
{
    i2cInitialised = msg.isAlive;
}

void ImuDmpWritingServer::writeDmp(void)
{
    /* This address is the start address of DMP code */
    /* It is coming from InvenSense */
    const uint8_t startAddressMsb = 0x04;
    const uint8_t startAddressLsb = 0x00;

    uint8_t bank = 0;
    uint8_t byteAddressInBank = 0;
    uint8_t chunkAddressInBank = 0;
    uint8_t indexInChunk = 0;
    std::vector<uint8_t> data;
    bool writeSuccess = false;

    bool writeRequest = imuDmpWritingServer->getActionServerHandle()->acceptNewGoal()->write;

    result.success = true;

    RCLCPP_INFO(get_logger(), "Started writing DMP code.");

    for (uint16_t byte = 0; byte < DMP_CODE_SIZE; ++byte)
    {
        indexInChunk = byte % MPU6050_CHUNK_SIZE;
        byteAddressInBank = byte % MPU6050_BANK_SIZE;
        bank = (byte - byteAddressInBank) / MPU6050_BANK_SIZE;

        data.push_back(dmp_memory[byte]);

        /* The chunk is full and ready to be written or we reached the end of the DMP code */
        if ((indexInChunk == (MPU6050_CHUNK_SIZE - 1)) || (byte == DMP_CODE_SIZE - 1))
        {
            chunkAddressInBank = byteAddressInBank - indexInChunk;
            
            if (!writeData(bank, chunkAddressInBank, data))
            {
                RCLCPP_ERROR(get_logger(), "Failed to write DMP code: chunk at address %u of bank %u not written!", chunkAddressInBank, bank);
                result.success = false;
                imuDmpWritingServer->getActionServerHandle()->setAborted(result);
                return;
            }

            data.clear();
        }
    }

    writeSuccess = writeByteInRegister(MPU6050_DMP_START_ADDRESS_H_REGISTER, startAddressMsb);
    writeSuccess &= writeByteInRegister(MPU6050_DMP_START_ADDRESS_L_REGISTER, startAddressLsb);
    if (!writeSuccess)
    {
        RCLCPP_ERROR(get_logger(), "Failed to write DMP code: start address not written!");
        result.success = false;
        imuDmpWritingServer->getActionServerHandle()->setAborted(result);
        return;
    }

    RCLCPP_INFO(get_logger(), "Successfully wrote DMP code.");
    imuDmpWritingServer->getActionServerHandle()->setSucceeded(result);
}

bool ImuDmpWritingServer::writeData(uint8_t bank, uint8_t addressInBank, std::vector<uint8_t> data)
{
    if (addressInBank == 0)
    {
        /* A new bank is starting */
        if (!writeByteInRegister(MPU6050_BANK_SELECTION_REGISTER, bank))
        {
            return false;
        }
    }

    if (!writeByteInRegister(MPU6050_ADDRESS_IN_BANK_REGISTER, addressInBank))
    {
        return false;
    }

    if (!writeDataBlock(MPU6050_READ_WRITE_REGISTER, data))
    {
        return false;
    }

    return true;
}

bool ImuDmpWritingServer::writeByteInRegister(uint8_t registerToWrite, uint8_t value)
{
    hal_pigpio::hal_pigpioI2cWriteByteData i2cWriteByteDataSrv;

    i2cWriteByteDataSrv.request.handle = imuHandle;
    i2cWriteByteDataSrv.request.deviceRegister = registerToWrite;
    i2cWriteByteDataSrv.request.value = value;

    imuDmpClients->getWriteByteDataClientHandle()->call(i2cWriteByteDataSrv);

    return i2cWriteByteDataSrv.response.hasSucceeded;
}

bool ImuDmpWritingServer::writeDataBlock(uint8_t registerToWrite, std::vector<uint8_t> data)
{
    hal_pigpio::hal_pigpioI2cWriteBlockData i2cWriteBlockDataSrv;

    i2cWriteBlockDataSrv.request.handle = imuHandle;
    i2cWriteBlockDataSrv.request.deviceRegister = registerToWrite;
    i2cWriteBlockDataSrv.request.length = data.size();

    for (uint8_t index = 0; index < data.size(); index++)
    {
        i2cWriteBlockDataSrv.request.dataBlock.push_back(data.at(index));
    }

    imuDmpClients->getWriteBlockDataClientHandle()->call(i2cWriteBlockDataSrv);

    return i2cWriteBlockDataSrv.response.hasSucceeded;
}

void ImuDmpWritingServer::starts(void)
{
    isStarted = true;
}

bool ImuDmpWritingServer::isNotStarted(void)
{
    return !isStarted;
}

/*int main(int argc, char **argv)
{
    ros::init(argc, argv, "hal_imuDmpWritingServer");
    ros::NodeHandle node;

    ImuDmpWritingClientsRos imuServiceClients(&node);
    ImuDmpWritingActionServerRos imuWriteDmpServer(&node);
    ImuDmpWritingServerSubscribersRos imuDmpWritingServerSubscribers(&node);

    ImuDmpWritingServer imuDmpWritingServer(&imuWriteDmpServer, &imuServiceClients, &imuDmpWritingServerSubscribers);

    RCLCPP_INFO(get_logger(), "imuDmpWritingServer node waiting for I2C communication to be ready...");
    while (ros::ok())
    {
        if (imuDmpWritingServer.isNotStarted() && imuDmpWritingServer.isI2cInitialised())
        {
            imuDmpWritingServer.getI2cHandle();
            RCLCPP_INFO(get_logger(), "imuDmpWritingServer I2C communication ready.");
            imuDmpWritingServer.startServer();
            imuDmpWritingServer.starts();
        }

        ros::spinOnce();
    }

    return 0;
}*/