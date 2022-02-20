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
                                                      i2cCloseClientRos(node->serviceClient<hal_pigpio::hal_pigpioI2cClose>("hal_pigpioI2cClose")),
                                                      i2cReadByteDataClientRos(node->serviceClient<hal_pigpio::hal_pigpioI2cReadByteData>("hal_pigpioI2cReadByteData")),
                                                      i2cWriteByteDataClientRos(node->serviceClient<hal_pigpio::hal_pigpioI2cWriteByteData>("hal_pigpioI2cWriteByteData"))
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

ros::ServiceClient *ImuClientsRos::getReadByteDataClientHandle()
{
    return &i2cReadByteDataClientRos;
}

ros::ServiceClient *ImuClientsRos::getWriteByteDataClientHandle()
{
    return &i2cWriteByteDataClientRos;
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

void Imu::init(void)
{
    this->initI2cCommunication();
    this->writeDmp();
    this->calibrateAccelerometer();
    this->enableGyroCalibrationOnDMP();
}

void Imu::initI2cCommunication(void)
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

void Imu::calibrateAccelerometer(void)
{
}

void Imu::enable6AxisQuaternion(void)
{
    const unsigned char dmp6AxisQuaternionEnable[DMP_FEATURE_6X_LP_QUAT_SIZE] = {0x20, 0x28, 0x30, 0x38};
    bool writeSuccess = false;
    uint8_t imuRegister = 0;

    const uint8_t lsb6AxisQuaternionAddress = (uint8_t)(DMP_FEATURE_6X_LP_QUAT >> 8);
    const uint8_t msb6AxisQuaternionAddress = (uint8_t)(DMP_FEATURE_6X_LP_QUAT & 0xFF);
    writeSuccess = writeByteInRegister(imuRegister, lsb6AxisQuaternionAddress);
    if (writeSuccess)
    {
        writeSuccess = writeByteInRegister(imuRegister, msb6AxisQuaternionAddress);
        if (!writeSuccess)
        {
            ROS_ERROR("Error while enabling 6 axis quaternions on DMP!");
        }
    }
    else
    {
        ROS_ERROR("Error while enabling 6 axis quaternions on DMP!");
    }

    for (uint8_t index; index < DMP_FEATURE_6X_LP_QUAT_SIZE; ++index)
    {
        writeSuccess = writeByteInRegister(imuRegister, dmp6AxisQuaternionEnable[index]);
        if (writeSuccess)
        {
            ROS_INFO("Sucessfully enabled 6 axis quaternions on DMP!");
        }
        else
        {
            ROS_ERROR("Error while enabling 6 axis quaternions on DMP!");
            break;
        }
    }
}

void Imu::enableGyroCalibrationOnDMP(void)
{
}

bool Imu::writeByteInRegister(uint8_t chipRegister, uint8_t value)
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
    ros::init(argc, argv, "hal_imu");
    ros::NodeHandle node;

    ImuPublisherRos imuMessagePublisherRos(&node);
    ImuServersRos imuServiceServersRos(&node);
    ImuClientsRos imuServiceClientsRos(&node);

    Imu imu(&imuMessagePublisherRos, &imuServiceServersRos, &imuServiceClientsRos);
    imu.init();

    ros::Rate loop_rate(200);

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}