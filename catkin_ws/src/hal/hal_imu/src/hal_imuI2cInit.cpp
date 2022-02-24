
imuServers->advertiseGetHandleService(this);

bool Imu::getHandle(hal_imu::hal_imuGetHandle::Request &req,
                    hal_imu::hal_imuGetHandle::Response &res)
{
    res.handle = imuHandle;
    return true;
}

/* Services servers interface implementation */
ImuServersRos::ImuServersRos(ros::NodeHandle *node) : nodeHandle(node)
{
}

void ImuServersRos::advertiseGetHandleService(Imu *imu)
{
    imuGetHandleServerRos = nodeHandle->advertiseService("hal_imuGetHandle", &Imu::getHandle, imu);
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

ImuServersRos imuServiceServersRos(&node);