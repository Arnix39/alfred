#ifndef HAL_PIGPIO
#define HAL_PIGPIO

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

// Pigpio library
#include "pigpiod_if2.h"

#include "hal_pigpioImu.hpp"
#include "hal_pigpioI2c.hpp"
#include "hal_pigpioInit.hpp"
#include "hal_pigpioInput.hpp"
#include "hal_pigpioOutput.hpp"

class Pigpio : public rclcpp_lifecycle::LifecycleNode
{
private:
    int pigpioHandle;
    int32_t i2cHandle;
    /* pigpioI2c */
    rclcpp::Service<hal_pigpio_interfaces::srv::HalPigpioI2cOpen>::SharedPtr i2cOpenService;
    rclcpp::Service<hal_pigpio_interfaces::srv::HalPigpioI2cClose>::SharedPtr i2cCloseService;
    rclcpp::Service<hal_pigpio_interfaces::srv::HalPigpioI2cReadByteData>::SharedPtr i2cReadByteDataService;
    rclcpp::Service<hal_pigpio_interfaces::srv::HalPigpioI2cReadWordData>::SharedPtr i2cReadWordDataService;
    rclcpp::Service<hal_pigpio_interfaces::srv::HalPigpioI2cReadBlockData>::SharedPtr i2cReadBlockDataService;
    rclcpp::Service<hal_pigpio_interfaces::srv::HalPigpioI2cWriteByteData>::SharedPtr i2cWriteByteDataService;
    rclcpp::Service<hal_pigpio_interfaces::srv::HalPigpioI2cWriteWordData>::SharedPtr i2cWriteWordDataService;
    rclcpp::Service<hal_pigpio_interfaces::srv::HalPigpioI2cWriteBlockData>::SharedPtr i2cWriteBlockDataService;
    /* pigpioImu */
    Quaternions quaternions;
    Angles angles;
    bool isImuReady;
    rclcpp::TimerBase::SharedPtr readQuaternionsAndPublishAnglesTimer;
    rclcpp::Service<hal_pigpio_interfaces::srv::HalPigpioI2cImuReading>::SharedPtr imuReadingService;
    rclcpp::Publisher<hal_pigpio_interfaces::msg::HalPigpioAngles>::SharedPtr anglesPublisher;
    //
    rclcpp::Service<hal_pigpio_interfaces::srv::HalPigpioGetMode>::SharedPtr getModeService;
    rclcpp::Service<hal_pigpio_interfaces::srv::HalPigpioSetInputMode>::SharedPtr setInputModeService;
    rclcpp::Service<hal_pigpio_interfaces::srv::HalPigpioSetOutputMode>::SharedPtr setOutputModeService;
    rclcpp::Service<hal_pigpio_interfaces::srv::HalPigpioSetPullUp>::SharedPtr setPullUpService;
    rclcpp::Service<hal_pigpio_interfaces::srv::HalPigpioSetPullDown>::SharedPtr setPullDownService;
    rclcpp::Service<hal_pigpio_interfaces::srv::HalPigpioClearResistor>::SharedPtr clearResistorService;
    //
    rclcpp::Publisher<hal_pigpio_interfaces::msg::HalPigpioEdgeChange>::SharedPtr gpioEdgeChangePub;
    rclcpp::Publisher<hal_pigpio_interfaces::msg::HalPigpioEncoderCount>::SharedPtr gpioEncoderCountPub;
    rclcpp::Service<hal_pigpio_interfaces::srv::HalPigpioReadGpio>::SharedPtr readGpioService;
    rclcpp::Service<hal_pigpio_interfaces::srv::HalPigpioSetCallback>::SharedPtr setCallbackService;
    rclcpp::Service<hal_pigpio_interfaces::srv::HalPigpioSetEncoderCallback>::SharedPtr setEncoderCallbackService;
    rclcpp::Service<hal_pigpio_interfaces::srv::HalPigpioSetMotorDirection>::SharedPtr setMotorDirectionService;
    std::vector<uint> callbackList;
    std::vector<Motor> motors;
    inline void gpioEdgeChangeCallback(int handle, unsigned gpioId, unsigned edgeChangeType, uint32_t timeSinceBoot_us);
    static void c_gpioEdgeChangeCallback(int handle, unsigned gpioId, unsigned edgeChangeType, uint32_t timeSinceBoot_us, void *userData);
    inline void gpioEncoderEdgeChangeCallback(int handle, unsigned gpioId, unsigned edgeChangeType, uint32_t timeSinceBoot_us);
    static void c_gpioEncoderEdgeChangeCallback(int handle, unsigned gpioId, unsigned edgeChangeType, uint32_t timeSinceBoot_us, void *userData);
    //
    rclcpp::Client<hal_pigpio_interfaces::srv::HalPigpioGetMode>::SharedPtr getModeClient;
    rclcpp::Service<hal_pigpio_interfaces::srv::HalPigpioSetPwmDutycycle>::SharedPtr setPwmDutycycleService;
    rclcpp::Service<hal_pigpio_interfaces::srv::HalPigpioSetPwmFrequency>::SharedPtr setPwmFrequencyService;
    rclcpp::Service<hal_pigpio_interfaces::srv::HalPigpioSetGpioHigh>::SharedPtr setGpioHighService;
    rclcpp::Service<hal_pigpio_interfaces::srv::HalPigpioSetGpioLow>::SharedPtr setGpioLowService;
    rclcpp::Service<hal_pigpio_interfaces::srv::HalPigpioSendTriggerPulse>::SharedPtr sendTriggerPulseService;
    //
    rclcpp::TimerBase::SharedPtr encoderCountTimer;

public:
    Pigpio();
    ~Pigpio();
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State &);
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State &);
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &);
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &);
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &);
    /* pigpioI2c */
    void i2cOpen(const std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioI2cOpen::Request> request,
                 std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioI2cOpen::Response> response);
    void i2cClose(const std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioI2cClose::Request> request,
                  std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioI2cClose::Response> response);
    void i2cReadByteData(const std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioI2cReadByteData::Request> request,
                         std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioI2cReadByteData::Response> response);
    void i2cReadWordData(const std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioI2cReadWordData::Request> request,
                         std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioI2cReadWordData::Response> response);
    void i2cReadBlockData(const std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioI2cReadBlockData::Request> request,
                          std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioI2cReadBlockData::Response> response);
    void i2cWriteByteData(const std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioI2cWriteByteData::Request> request,
                          std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioI2cWriteByteData::Response> response);
    void i2cWriteWordData(const std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioI2cWriteWordData::Request> request,
                          std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioI2cWriteWordData::Response> response);
    void i2cWriteBlockData(const std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioI2cWriteBlockData::Request> request,
                           std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioI2cWriteBlockData::Response> response);
    /* pigpioImu */
    void readQuaternions(void);
    void computeQuaternions(char (&data)[MPU6050_DMP_FIFO_QUAT_SIZE]);
    void publishAngles(void);
    void computeAngles(void);
    void readQuaternionsAndPublishAngles(void);
    void resetFifo(void);
    uint16_t readFifoCount(void);
    bool isFifoOverflowed(void);
    void i2cImuReading(const std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioI2cImuReading::Request> request,
                       std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioI2cImuReading::Response> response);
    /* pigpioInit */
    void getMode(const std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioGetMode::Request> request,
                 std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioGetMode::Response> response);
    void setInputMode(const std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioSetInputMode::Request> request,
                      std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioSetInputMode::Response> response);
    void setOutputMode(const std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioSetOutputMode::Request> request,
                       std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioSetOutputMode::Response> response);
    void setPullUp(const std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioSetPullUp::Request> request,
                   std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioSetPullUp::Response> response);
    void setPullDown(const std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioSetPullDown::Request> request,
                     std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioSetPullDown::Response> response);
    void clearResistor(const std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioClearResistor::Request> request,
                       std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioClearResistor::Response> response);
    //
    void readGpio(const std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioReadGpio::Request> request,
                  std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioReadGpio::Response> response);
    void setCallback(const std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioSetCallback::Request> request,
                     std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioSetCallback::Response> response);
    void setEncoderCallback(const std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioSetEncoderCallback::Request> request,
                            std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioSetEncoderCallback::Response> response);
    void setMotorDirection(const std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioSetMotorDirection::Request> request,
                           std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioSetMotorDirection::Response> response);
    void publishEncoderCount(void);
    //
    void setPwmDutycycle(const std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioSetPwmDutycycle::Request> request,
                         std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioSetPwmDutycycle::Response> response);
    void setPwmFrequency(const std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioSetPwmFrequency::Request> request,
                         std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioSetPwmFrequency::Response> response);
    void setGpioHigh(const std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioSetGpioHigh::Request> request,
                    std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioSetGpioHigh::Response> response);
    void setGpioLow(const std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioSetGpioLow::Request> request,
                    std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioSetGpioLow::Response> response);
    void sendTriggerPulse(const std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioSendTriggerPulse::Request> request,
                          std::shared_ptr<hal_pigpio_interfaces::srv::HalPigpioSendTriggerPulse::Response> response);
};

#endif