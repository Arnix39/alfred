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
}

void ImuDmpWritingServer::writeDmp(void)
{
    uint8_t bank = 0;
    uint8_t addressInBank = 0;
    uint8_t byteData = 0;

    hal_imu::hal_imuGetHandle i2cGetHandleSrv;
    imuDmpClients->getGetHandleClientHandle()->call(i2cGetHandleSrv);
    imuHandle = i2cGetHandleSrv.response.handle;

    bool writeRequest = imuDmpWritingServer->getActionServerHandle()->acceptNewGoal()->write;
    ROS_INFO("Goal received.");

    result.success = true;

    for (uint16_t byte = 0; byte <= DMP_CODE_SIZE; byte++)
    {
        bool writeSuccess = false;

        byteData = dmp_memory[byte];
        addressInBank = byte % MPU6050_BANK_SIZE;
        bank = (byte - addressInBank) / MPU6050_BANK_SIZE;

        if (addressInBank == 0)
        {
            feedback.bank = bank;
            ROS_INFO("Writing bank %u...", bank);
            imuDmpWritingServer->getActionServerHandle()->publishFeedback(feedback);
        }

        writeSuccess = writeByte(bank, addressInBank, byteData);
        if (!writeSuccess)
        {
            ROS_ERROR("Failed to write byte at address %u of bank %u...", addressInBank, bank);
            result.success = false;
            break;
        }
    }

    if (result.success)
    {
        imuDmpWritingServer->getActionServerHandle()->setSucceeded(result);
    }
    else
    {
        imuDmpWritingServer->getActionServerHandle()->setAborted(result);
    }
}

bool ImuDmpWritingServer::writeByte(uint8_t bank, uint8_t addressInBank, uint8_t value)
{
    bool writeSuccess = false;
    if (addressInBank == 0)
    {
        /* A new bank is starting */
        ROS_INFO("Setting new bank.");
        writeSuccess = writeByteInRegister(MPU6050_BANK_SELECTION_REGISTER, bank);
        if (!writeSuccess)
        {
            return false;
        }
    }

    ROS_INFO("Setting address in bank.");
    writeSuccess = writeByteInRegister(MPU6050_ADDRESS_IN_BANK_REGISTER, addressInBank);
    if (!writeSuccess)
    {
        return false;
    }

    ROS_INFO("Setting data to write.");
    writeSuccess = writeByteInRegister(MPU6050_READ_WRITE_REGISTER, value);
    if (writeSuccess)
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool ImuDmpWritingServer::writeByteInRegister(uint8_t chipRegister, uint8_t value)
{
    hal_pigpio::hal_pigpioI2cWriteByteData i2cWriteByteDataSrv;
    hal_pigpio::hal_pigpioI2cReadByteData i2cReadByteDataSrv;

    i2cWriteByteDataSrv.request.handle = imuHandle;
    i2cReadByteDataSrv.request.handle = imuHandle;

    i2cWriteByteDataSrv.request.deviceRegister = chipRegister;
    i2cWriteByteDataSrv.request.value = value;

    ROS_INFO("Writing byte.");
    imuDmpClients->getWriteByteDataClientHandle()->call(i2cWriteByteDataSrv);

    if (i2cWriteByteDataSrv.response.hasSucceeded)
    {
        ROS_INFO("Reading byte.");
        imuDmpClients->getReadByteDataClientHandle()->call(i2cReadByteDataSrv);
        if (i2cReadByteDataSrv.response.hasSucceeded && (i2cReadByteDataSrv.response.value == value))
        {
            ROS_INFO("Byte check OK.");
            return true;
        }
        else
        {
            ROS_ERROR("Byte check failed!");
        }
    }
    else
    {
        ROS_ERROR("Failed to write byte!");
    }

    return false;
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