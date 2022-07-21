#include "hal_pigpio.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

Pigpio::Pigpio() : rclcpp_lifecycle::LifecycleNode("hal_pigpio_node"),
                   pigpioHandle(PI_NO_HANDLE),
                   i2cHandle(PI_NO_HANDLE),
                   quaternions({0.0, 0.0, 0.0, 0.0}),
                   angles({0.0, 0.0, 0.0}),
                   isImuReady(false),
                   callbackList({}),
                   motors({})
{
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Pigpio::on_configure(const rclcpp_lifecycle::State & previous_state)
{
    pigpioHandle = pigpio_start(NULL, NULL);
    if (pigpioHandle < 0)
    {
        RCLCPP_ERROR(get_logger(),"Pigpio daemon not running!");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(get_logger(), "Pigpio handle: %d.", pigpioHandle);

    i2cOpenService = this->create_service<hal_pigpio_interfaces::srv::HalPigpioI2cOpen>("hal_pigpioI2cOpen", std::bind(&Pigpio::i2cOpen, this, _1, _2));
    i2cCloseService = this->create_service<hal_pigpio_interfaces::srv::HalPigpioI2cClose>("hal_pigpioI2cClose", std::bind(&Pigpio::i2cClose, this, _1, _2));
    i2cReadByteDataService = this->create_service<hal_pigpio_interfaces::srv::HalPigpioI2cReadByteData>("hal_pigpioI2cReadByteData", std::bind(&Pigpio::i2cReadByteData, this, _1, _2));
    i2cReadWordDataService = this->create_service<hal_pigpio_interfaces::srv::HalPigpioI2cReadWordData>("hal_pigpioI2cReadWordData", std::bind(&Pigpio::i2cReadWordData, this, _1, _2));
    i2cReadBlockDataService = this->create_service<hal_pigpio_interfaces::srv::HalPigpioI2cReadBlockData>("hal_pigpioI2cReadBlockData", std::bind(&Pigpio::i2cReadBlockData, this, _1, _2));
    i2cWriteByteDataService = this->create_service<hal_pigpio_interfaces::srv::HalPigpioI2cWriteByteData>("hal_pigpioI2cWriteByteData", std::bind(&Pigpio::i2cWriteByteData, this, _1, _2));
    i2cWriteWordDataService = this->create_service<hal_pigpio_interfaces::srv::HalPigpioI2cWriteWordData>("hal_pigpioI2cWriteWordData", std::bind(&Pigpio::i2cWriteWordData, this, _1, _2));
    i2cWriteBlockDataService = this->create_service<hal_pigpio_interfaces::srv::HalPigpioI2cWriteBlockData>("hal_pigpioI2cWriteBlockData", std::bind(&Pigpio::i2cWriteBlockData, this, _1, _2));
    imuReadingService = this->create_service<hal_pigpio_interfaces::srv::HalPigpioI2cImuReading>("hal_pigpioI2cImuReading", std::bind(&Pigpio::i2cImuReading, this, _1, _2));
    setInputModeService = this->create_service<hal_pigpio_interfaces::srv::HalPigpioSetInputMode>("hal_pigpioSetInputMode", std::bind(&Pigpio::setInputMode, this, _1, _2));
    setOutputModeService = this->create_service<hal_pigpio_interfaces::srv::HalPigpioSetOutputMode>("hal_pigpioSetOutputMode", std::bind(&Pigpio::setOutputMode, this, _1, _2));
    setPullUpService = this->create_service<hal_pigpio_interfaces::srv::HalPigpioSetPullUp>("hal_pigpioSetPullUp", std::bind(&Pigpio::setPullUp, this, _1, _2));
    setPullDownService = this->create_service<hal_pigpio_interfaces::srv::HalPigpioSetPullDown>("hal_pigpioSetPullDown", std::bind(&Pigpio::setPullDown, this, _1, _2));
    clearResistorService = this->create_service<hal_pigpio_interfaces::srv::HalPigpioClearResistor>("hal_pigpioClearResistor", std::bind(&Pigpio::clearResistor, this, _1, _2));
    getModeService = this->create_service<hal_pigpio_interfaces::srv::HalPigpioGetMode>("hal_pigpioGetMode", std::bind(&Pigpio::getMode, this, _1, _2));
    readGpioService = this->create_service<hal_pigpio_interfaces::srv::HalPigpioReadGpio>("hal_pigpioReadGpio", std::bind(&Pigpio::readGpio, this, _1, _2));
    setCallbackService = this->create_service<hal_pigpio_interfaces::srv::HalPigpioSetCallback>("hal_pigpioSetCallback", std::bind(&Pigpio::setCallback, this, _1, _2));
    setEncoderCallbackService = this->create_service<hal_pigpio_interfaces::srv::HalPigpioSetEncoderCallback>("hal_pigpioSetEncoderCallback", std::bind(&Pigpio::setEncoderCallback, this, _1, _2));
    setMotorDirectionService = this->create_service<hal_pigpio_interfaces::srv::HalPigpioSetMotorDirection>("hal_pigpioSetMotorDirection", std::bind(&Pigpio::setMotorDirection, this, _1, _2));
    setPwmDutycycleService = this->create_service<hal_pigpio_interfaces::srv::HalPigpioSetPwmDutycycle>("hal_pigpioSetPwmDutycycle", std::bind(&Pigpio::setPwmDutycycle, this, _1, _2));
    setPwmFrequencyService = this->create_service<hal_pigpio_interfaces::srv::HalPigpioSetPwmFrequency>("hal_pigpioSetPwmFrequency", std::bind(&Pigpio::setPwmFrequency, this, _1, _2));
    setGpioHighService = this->create_service<hal_pigpio_interfaces::srv::HalPigpioSetGpioHigh>("hal_pigpioSetGpioHigh", std::bind(&Pigpio::setGpioHigh, this, _1, _2));
    setGpioLowService = this->create_service<hal_pigpio_interfaces::srv::HalPigpioSetGpioLow>("hal_pigpioSetGpioLow", std::bind(&Pigpio::setGpioLow, this, _1, _2));
    sendTriggerPulseService = this->create_service<hal_pigpio_interfaces::srv::HalPigpioSendTriggerPulse>("hal_pigpioSendTriggerPulse", std::bind(&Pigpio::sendTriggerPulse, this, _1, _2));

    gpioEdgeChangePub = this->create_publisher<hal_pigpio_interfaces::msg::HalPigpioEdgeChange>("gpioEdgeChange", 1000);
    gpioEncoderCountPub = this->create_publisher<hal_pigpio_interfaces::msg::HalPigpioEncoderCount>("hal_pigpioEncoderCount", 1000);
    anglesPublisher = this->create_publisher<hal_pigpio_interfaces::msg::HalPigpioAngles>("hal_pigpioAngles", 1000);

    readQuaternionsAndPublishAnglesTimer = create_wall_timer(5ms, std::bind(&Pigpio::readQuaternionsAndPublishAngles, this));
    encoderCountTimer = create_wall_timer(5ms, std::bind(&Pigpio::publishEncoderCount, this));

    RCLCPP_INFO(get_logger(), "hal_pigpio node configured!");

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Pigpio::on_activate(const rclcpp_lifecycle::State & previous_state)
{
    gpioEdgeChangePub->on_activate();
    gpioEncoderCountPub->on_activate();
    anglesPublisher->on_activate();

    RCLCPP_INFO(get_logger(), "hal_pigpio node activated!");

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Pigpio::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
    gpioEdgeChangePub->on_deactivate();
    gpioEncoderCountPub->on_deactivate();
    anglesPublisher->on_deactivate();

    isImuReady = false;
    quaternions = {0.0, 0.0, 0.0, 0.0};
    angles = {0.0, 0.0, 0.0};

    i2cHandle = PI_NO_HANDLE;

    RCLCPP_INFO(get_logger(), "hal_pigpio node deactivated!");

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Pigpio::on_cleanup(const rclcpp_lifecycle::State & previous_state)
{
    for (uint callbackId : callbackList)
    {
        callback_cancel(callbackId);
    }
    callbackList.clear();

    motors.clear();

    gpioEdgeChangePub.reset();
    gpioEncoderCountPub.reset();
    anglesPublisher.reset();

    readQuaternionsAndPublishAnglesTimer.reset();
    encoderCountTimer.reset();

    if (pigpioHandle >= 0)
    {
        RCLCPP_INFO(get_logger(), "Releasing pigpio daemon.");
        pigpio_stop(pigpioHandle);
        pigpioHandle = PI_NO_HANDLE;
    }

    RCLCPP_INFO(get_logger(), "hal_pigpio node unconfigured!");

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Pigpio::on_shutdown(const rclcpp_lifecycle::State & previous_state)
{
    for (uint callbackId : callbackList)
    {
        callback_cancel(callbackId);
    }
    callbackList.clear();

    motors.clear();

    gpioEdgeChangePub.reset();
    gpioEncoderCountPub.reset();
    anglesPublisher.reset();

    readQuaternionsAndPublishAnglesTimer.reset();
    encoderCountTimer.reset();

    if (pigpioHandle >= 0)
    {
        RCLCPP_INFO(get_logger(), "Releasing pigpio daemon.");
        pigpio_stop(pigpioHandle);
        pigpioHandle = PI_NO_HANDLE;
    }

    RCLCPP_INFO(get_logger(), "hal_pigpio node shutdown!");

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Pigpio::on_error(const rclcpp_lifecycle::State & previous_state)
{    
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
}