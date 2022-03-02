#include "hal_imuDmpWritingServer.hpp"
#include "hal_imuDmpWritingServerInterfaces.hpp"
#include "hal_imuMPU6050.hpp"

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
ImuDmpWritingClientsRos::ImuDmpWritingClientsRos(ros::NodeHandle *node) : i2cReadByteDataClientRos(node->serviceClient<hal_pigpio::hal_pigpioI2cReadByteData>("hal_pigpioI2cReadByteData")),
                                                                          i2cWriteByteDataClientRos(node->serviceClient<hal_pigpio::hal_pigpioI2cWriteByteData>("hal_pigpioI2cWriteByteData")),
                                                                          i2cWriteBlockDataClientRos(node->serviceClient<hal_pigpio::hal_pigpioI2cWriteBlockData>("hal_pigpioI2cWriteBlockData")),
                                                                          i2cGetHandleClientRos(node->serviceClient<hal_imu::hal_imuGetHandle>("hal_imuGetHandle"))
{
}

ros::ServiceClient *ImuDmpWritingClientsRos::getReadByteDataClientHandle()
{
    return &i2cReadByteDataClientRos;
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

/* IMU DMP implementation */
ImuDmpWritingServer::ImuDmpWritingServer(ImuDmpWritingActionServer *imuWriteDmpServer, ImuDmpWritingClients *imuDmpServiceClients) : imuDmpWritingServer(imuWriteDmpServer),
                                                                                                                                     imuDmpClients(imuDmpServiceClients)
{
    imuDmpWritingServer->registerCallback(this);
    imuDmpWritingServer->getActionServerHandle()->start();
    ROS_INFO("Action server started.");

    hal_imu::hal_imuGetHandle i2cGetHandleSrv;
    imuDmpClients->getGetHandleClientHandle()->call(i2cGetHandleSrv);
    imuHandle = i2cGetHandleSrv.response.handle;
}

void ImuDmpWritingServer::writeDmp(void)
{
    uint8_t bank = 0;
    uint8_t byteAddressInBank = 0;
    uint8_t chunkAddressInBank = 0;
    uint8_t indexInChunk = 0;
    std::vector<uint8_t> data;
    bool readyToWriteChunk = false;

    bool writeRequest = imuDmpWritingServer->getActionServerHandle()->acceptNewGoal()->write;

    result.success = true;

    ROS_INFO("Started writing DMP code.");

    for (uint16_t byte = 0; byte < DMP_CODE_SIZE; byte++)
    {
        bool writeSuccess = false;
        indexInChunk = byte % MPU6050_CHUNK_SIZE;
        byteAddressInBank = byte % MPU6050_BANK_SIZE;
        bank = (byte - byteAddressInBank) / MPU6050_BANK_SIZE;

        data.push_back(dmp_memory[byte]);

        if (byteAddressInBank == 0)
        {
            feedback.bank = bank;
            imuDmpWritingServer->getActionServerHandle()->publishFeedback(feedback);
        }

        /* The chunk is full and ready to be written or we reached the end of the DMP code */
        if ((indexInChunk == (MPU6050_CHUNK_SIZE - 1)) || (byte == DMP_CODE_SIZE - 1))
        {
            chunkAddressInBank = byteAddressInBank - indexInChunk;
            writeSuccess = writeData(bank, chunkAddressInBank, data);
            if (!writeSuccess)
            {
                ROS_ERROR("Failed to write chunk at address %u of bank %u!", chunkAddressInBank, bank);
                result.success = false;
                break;
            }

            data.clear();
        }
    }

    writeDmpStartAddress();

    if (result.success)
    {
        ROS_INFO("Wrote successfully DMP code.");
        imuDmpWritingServer->getActionServerHandle()->setSucceeded(result);
    }
    else
    {
        ROS_ERROR("Failed to write DMP code!");
        imuDmpWritingServer->getActionServerHandle()->setAborted(result);
    }
}

bool ImuDmpWritingServer::writeDmpStartAddress(void)
{
    /* This address is the start address of DMP code */
    /* It is coming from InvenSense */
    static const unsigned short sStartAddress = 0x0400;

    bool writeSuccess = false;
    const uint8_t lsbAddress = (uint8_t)(sStartAddress >> 8);
    const uint8_t msbAddress = (uint8_t)(sStartAddress & 0xFF);
    writeSuccess = writeByteInRegister(MPU6050_DMP_CONFIGURATION_REGISTER, lsbAddress);
    if (writeSuccess)
    {
        writeSuccess = writeByteInRegister(MPU6050_DMP_CONFIGURATION_REGISTER, msbAddress);
        if (writeSuccess)
        {
            return true;
        }
        else
        {
            ROS_ERROR("Failed to write data address MSB!");
            return false;
        }
    }
    else
    {
        ROS_ERROR("Failed to write data address LSB!");
        return false;
    }
}

bool ImuDmpWritingServer::writeData(uint8_t bank, uint8_t addressInBank, std::vector<uint8_t> data)
{
    bool writeSuccess = false;
    if (addressInBank == 0)
    {
        /* A new bank is starting */
        writeSuccess = writeByteInRegister(MPU6050_BANK_SELECTION_REGISTER, bank);
        if (!writeSuccess)
        {
            ROS_ERROR("Failed to set new bank!");
            return false;
        }
    }

    writeSuccess = writeByteInRegister(MPU6050_ADDRESS_IN_BANK_REGISTER, addressInBank);
    if (!writeSuccess)
    {
        ROS_ERROR("Failed to set address in bank!");
        return false;
    }

    writeSuccess = writeDataBlock(MPU6050_READ_WRITE_REGISTER, data);
    if (writeSuccess)
    {
        return true;
    }
    else
    {
        ROS_ERROR("Failed to write data!");
        return false;
    }
}

bool ImuDmpWritingServer::writeByteInRegister(uint8_t registerToWrite, uint8_t value)
{
    hal_pigpio::hal_pigpioI2cWriteByteData i2cWriteByteDataSrv;

    i2cWriteByteDataSrv.request.handle = imuHandle;
    i2cWriteByteDataSrv.request.deviceRegister = registerToWrite;
    i2cWriteByteDataSrv.request.value = value;

    imuDmpClients->getWriteByteDataClientHandle()->call(i2cWriteByteDataSrv);

    if (i2cWriteByteDataSrv.response.hasSucceeded)
    {
        return true;
    }
    else
    {
        ROS_ERROR("Failed to write byte!");
        return false;
    }
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

    if (i2cWriteBlockDataSrv.response.hasSucceeded)
    {
        return true;
    }
    else
    {
        ROS_ERROR("Failed to write data block!");
        return false;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hal_imuDmpWritingServer");
    ros::NodeHandle node;

    ImuDmpWritingClientsRos imuServiceClients(&node);
    ImuDmpWritingActionServerRos imuWriteDmpServer(&node);

    ImuDmpWritingServer imuDmpWritingServer(&imuWriteDmpServer, &imuServiceClients);

    ros::spin();

    return 0;
}