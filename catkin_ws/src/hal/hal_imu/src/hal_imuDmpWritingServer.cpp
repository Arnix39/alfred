#include "hal_imuDmpWritingServer.hpp"
#include "hal_imuDmpWritingServerInterfaces.hpp"
#include "hal_imuMPU6050.hpp"

/* Action server interface implementation */
ImuActionServerRos::ImuActionServerRos(ros::NodeHandle *node) : nodeHandle(node),
                                                                imuWriteDmpServerRos(*nodeHandle, "imuDMPWriting", false)
{
}

void ImuActionServerRos::registerCallback(ImuDmpWritingServer *imuDmpWritingServer)
{
    imuWriteDmpServerRos.registerGoalCallback((std::function<void()>)std::bind(&ImuDmpWritingServer::writeDmp, imuDmpWritingServer));
}

imuActionServer_t *ImuActionServerRos::getActionServerHandle()
{
    return &imuWriteDmpServerRos;
}

/* Services interface implementation */
ImuClientsRos::ImuClientsRos(ros::NodeHandle *node) : i2cReadByteDataClientRos(node->serviceClient<hal_pigpio::hal_pigpioI2cReadByteData>("hal_pigpioI2cReadByteData")),
                                                      i2cWriteByteDataClientRos(node->serviceClient<hal_pigpio::hal_pigpioI2cWriteByteData>("hal_pigpioI2cWriteByteData")),
                                                      i2cGetHandleClientRos(node->serviceClient<hal_imu::hal_imuGetHandle>("hal_imuGetHandle"))
{
}

ros::ServiceClient *ImuClientsRos::getReadByteDataClientHandle()
{
    return &i2cReadByteDataClientRos;
}

ros::ServiceClient *ImuClientsRos::getWriteByteDataClientHandle()
{
    return &i2cWriteByteDataClientRos;
}

ros::ServiceClient *ImuClientsRos::getGetHandleClientHandle()
{
    return &i2cGetHandleClientRos;
}

/* IMU implementation */
ImuDmpWritingServer::ImuDmpWritingServer(ImuActionServer *imuWriteDmpServer, ImuClients *imuServiceClients) : imuDmpWritingServer(imuWriteDmpServer),
                                                                                                              imuClients(imuServiceClients)
{
    hal_imu::hal_imuGetHandle i2cGetHandleSrv;

    imuDmpWritingServer->registerCallback(this);
    imuDmpWritingServer->getActionServerHandle()->start();

    imuClients->getGetHandleClientHandle()->call(i2cGetHandleSrv);
    imuHandle = i2cGetHandleSrv.response.handle;
}

void ImuDmpWritingServer::writeDmp(void)
{
    uint8_t bank = 0;
    uint8_t addressInBank = 0;
    uint8_t byteData = 0;

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
            imuDmpWritingServer->getActionServerHandle()->publishFeedback(feedback);
        }

        writeSuccess = writeByte(bank, addressInBank, byteData);
        if (!writeSuccess)
        {
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
        writeSuccess = writeByteInRegister(MPU6050_BANK_SELECTION_REGISTER, bank);
        if (!writeSuccess)
        {
            return false;
        }
    }

    writeSuccess = writeByteInRegister(MPU6050_ADDRESS_IN_BANK_REGISTER, addressInBank);
    if (!writeSuccess)
    {
        return false;
    }

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
    imuClients->getWriteByteDataClientHandle()->call(i2cWriteByteDataSrv);

    if (i2cWriteByteDataSrv.response.hasSucceeded)
    {
        imuClients->getReadByteDataClientHandle()->call(i2cReadByteDataSrv);
        if (i2cReadByteDataSrv.response.hasSucceeded && (i2cReadByteDataSrv.response.value == value))
        {
            return true;
        }
    }

    return false;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hal_imuInit");
    ros::NodeHandle node;

    ImuClientsRos imuServiceClients(&node);
    ImuActionServerRos imuWriteDmpServer(&node);

    ImuDmpWritingServer imuDmpWritingServer(&imuWriteDmpServer, &imuServiceClients);

    ros::spin();

    return 0;
}