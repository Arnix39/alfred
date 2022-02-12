#include "hal_imuDmpWritingServer.hpp"
#include "hal_imuInterfaces.hpp"
#include "hal_imuMPU6050.hpp"

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
ImuDmpWritingServer::ImuDmpWritingServer(ros::NodeHandle *node, ImuClients *imuServiceClients) : imuDmpWritingServer(*node, "imuDMPWriting", boost::bind(&ImuDmpWritingServer::writeDmp, this, _1), false),
                                                                                                 nodeHandle(node),
                                                                                                 imuClients(imuServiceClients)
{
    imuDmpWritingServer.start();
}

void ImuDmpWritingServer::writeDmp(const hal_imu::hal_imuWriteDmpGoalConstPtr &goal)
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

        writeSuccess = writeByte(bank, addressInBank, byteData);
        if (!writeSuccess)
        {
            result.success = false;
            break;
        }
    }

    if (result.success)
    {
        imuDmpWritingServer.setSucceeded(result);
    }
    else
    {
        imuDmpWritingServer.setAborted(result);
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
    imuClients->getWriteByteDataClientHandle().call(i2cWriteByteDataSrv);

    if (i2cWriteByteDataSrv.response.hasSucceeded)
    {
        return true;
    }
    else
    {
        return false;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hal_imuInit");
    ros::NodeHandle node;

    ImuClientsRos imuServiceClients = ImuClientsRos(&node);

    ImuDmpWritingServer imuDmpWritingServer(&node, &imuServiceClients);

    ros::spin();

    return 0;
}