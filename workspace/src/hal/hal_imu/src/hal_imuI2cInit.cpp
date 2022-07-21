#include "hal_imuI2cInit.hpp"

ImuI2cInit::ImuI2cInit() : rclcpp_lifecycle::LifecycleNode("hal_imuI2cInit_node"),
                           imuHandle(MPU6050_I2C_NO_HANDLE)
{
}

ImuI2cInit::~ImuI2cInit()
{
    hal_pigpio::hal_pigpioI2cClose i2cCloseSrv;
    i2cCloseSrv.request.handle = imuHandle;
    imuI2cInitClients->getI2cCloseHandle()->call(i2cCloseSrv);
}

LifecycleCallbackReturn_t ImuI2cInit::on_configure(const rclcpp_lifecycle::State & previous_state);
LifecycleCallbackReturn_t ImuI2cInit::on_activate(const rclcpp_lifecycle::State & previous_state);
LifecycleCallbackReturn_t ImuI2cInit::on_deactivate(const rclcpp_lifecycle::State & previous_state);
LifecycleCallbackReturn_t ImuI2cInit::on_cleanup(const rclcpp_lifecycle::State & previous_state);
LifecycleCallbackReturn_t ImuI2cInit::on_shutdown(const rclcpp_lifecycle::State & previous_state);
LifecycleCallbackReturn_t ImuI2cInit::on_error(const rclcpp_lifecycle::State & previous_state);

bool ImuI2cInit::getHandle(hal_imu::hal_imuGetHandle::Request &req,
                           hal_imu::hal_imuGetHandle::Response &res)
{
    res.handle = imuHandle;
    return true;
}

void ImuI2cInit::initI2cCommunication(void)
{
    hal_pigpio::hal_pigpioI2cOpen i2cOpenSrv;

    i2cOpenSrv.request.bus = IMU_I2C_BUS;
    i2cOpenSrv.request.address = MPU6050_I2C_ADDRESS;

    imuI2cInitClients->getI2cOpenHandle()->call(i2cOpenSrv);
    if (i2cOpenSrv.response.hasSucceeded)
    {
        imuHandle = i2cOpenSrv.response.handle;
        RCLCPP_INFO(get_logger(), "Handle %u received for communication with device %u on bus %u.", imuHandle, i2cOpenSrv.request.address, i2cOpenSrv.request.bus);
    }
    else
    {
        RCLCPP_ERROR(get_logger(), "Unable to receive handle for communication with device %u on bus %u!", i2cOpenSrv.request.address, i2cOpenSrv.request.bus);
    }
}

/*int main(int argc, char **argv)
{
    ros::init(argc, argv, "hal_imuI2cInit");
    ros::NodeHandle node;
    ros::Timer heartbeatTimer;

    ImuI2cInitClientsRos imuI2cInitServiceClients(&node);
    ImuI2cInitServersRos imuI2cInitServers(&node);
    ImuI2cInitPublisherRos imuI2cInitPublishers(&node);
    ImuI2cInitSubscribersRos imuI2cInitSubscribers(&node);

    ImuI2cInit imuI2cInit(&imuI2cInitServiceClients, &imuI2cInitServers, &imuI2cInitPublishers, &imuI2cInitSubscribers);

    RCLCPP_INFO(get_logger(), "imuI2cInit node waiting for pigpio node to start...");
    while (ros::ok())
    {
        if (imuI2cInit.isNotStarted() && imuI2cInit.isPigpioNodeStarted())
        {
            RCLCPP_INFO(get_logger(), "imuI2cInit node initialising...");
            imuI2cInit.initI2cCommunication();
            imuI2cInit.starts();
            heartbeatTimer = node.createTimer(ros::Duration(0.1), &ImuI2cInit::publishHeartbeat, &imuI2cInit);
            RCLCPP_INFO(get_logger(), "imuI2cInit node initialised.");
        }

        ros::spinOnce();
    }

    return 0;
}*/