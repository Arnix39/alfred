#include "hal_motor_control.hpp"

using namespace std::placeholders;
using namespace std::chrono_literals;

MotorControl::MotorControl() : rclcpp_lifecycle::LifecycleNode("hal_motorControl_node"),
                               motorLeft(MOTOR_LEFT_PWM_A_GPIO, MOTOR_LEFT_PWM_B_GPIO,
                                         MOTOR_LEFT_ENCODER_CH_A_GPIO, MOTOR_LEFT_ENCODER_CH_B_GPIO,
                                         MOTOR_LEFT),
                               motorRight(MOTOR_RIGHT_PWM_A_GPIO, MOTOR_RIGHT_PWM_B_GPIO,
                                          MOTOR_RIGHT_ENCODER_CH_A_GPIO, MOTOR_RIGHT_ENCODER_CH_B_GPIO,
                                          MOTOR_RIGHT)
{
}

LifecycleCallbackReturn_t MotorControl::on_configure(const rclcpp_lifecycle::State & previous_state)
{
    gpioSetInputClient = this->create_client<hal_pigpio_interfaces::srv::HalPigpioSetInputMode>("hal_pigpioSetInputMode");
    gpioSetOutputClient = this->create_client<hal_pigpio_interfaces::srv::HalPigpioSetOutputMode>("hal_pigpioSetOutputMode");
    gpioSetEncoderCallbackClient = this->create_client<hal_pigpio_interfaces::srv::HalPigpioSetEncoderCallback>("hal_pigpioSetEncoderCallback");
    gpioSetPwmFrequencyClient = this->create_client<hal_pigpio_interfaces::srv::HalPigpioSetPwmFrequency>("hal_pigpioSetPwmFrequency");
    gpioSetPwmDutycycleClient = this->create_client<hal_pigpio_interfaces::srv::HalPigpioSetPwmDutycycle>("hal_pigpioSetPwmDutycycle");
    gpioSetMotorDirectionClient = this->create_client<hal_pigpio_interfaces::srv::HalPigpioSetMotorDirection>("hal_pigpioSetMotorDirection");

    motorControlPub = this->create_publisher<hal_motor_control_interfaces::msg::HalMotorControl>("motorsEncoderCountValue", 1000);

    motorControlECSub = this->create_subscription<hal_pigpio_interfaces::msg::HalPigpioEncoderCount>("hal_pigpioEncoderCount", 1000, std::bind(&MotorControl::pigpioEncoderCountCallback, this, _1));
    
    encoderCountsTimer = create_wall_timer(5ms, std::bind(&MotorControl::publishMessage, this));

    RCLCPP_INFO(get_logger(), "hal_motor_control node configured!");

    return LifecycleCallbackReturn_t::SUCCESS;
}

LifecycleCallbackReturn_t MotorControl::on_activate(const rclcpp_lifecycle::State & previous_state)
{
    motorControlPub->on_activate();

    configureMotor();

    RCLCPP_INFO(get_logger(), "hal_motor_control node activated!");

    return LifecycleCallbackReturn_t::SUCCESS;
}

LifecycleCallbackReturn_t MotorControl::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
    motorControlPub->on_deactivate();

    RCLCPP_INFO(get_logger(), "hal_motor_control node deactivated!");

    return LifecycleCallbackReturn_t::SUCCESS;
}

LifecycleCallbackReturn_t MotorControl::on_cleanup(const rclcpp_lifecycle::State & previous_state)
{
    motorControlPub.reset();
    encoderCountsTimer.reset();

    RCLCPP_INFO(get_logger(), "hal_motor_control node unconfigured!");

    return LifecycleCallbackReturn_t::SUCCESS;
}

LifecycleCallbackReturn_t MotorControl::on_shutdown(const rclcpp_lifecycle::State & previous_state)
{
    motorControlPub.reset();
    encoderCountsTimer.reset();

    RCLCPP_INFO(get_logger(), "hal_motor_control node shutdown!");

    return LifecycleCallbackReturn_t::SUCCESS;
}

LifecycleCallbackReturn_t MotorControl::on_error(const rclcpp_lifecycle::State & previous_state)
{
    return LifecycleCallbackReturn_t::FAILURE;
}


void MotorControl::configureMotor(void)
{
    motorLeft.configureGpios(gpioSetOutputClient, gpioSetInputClient, gpioSetEncoderCallbackClient, gpioSetPwmFrequencyClient);
    motorRight.configureGpios(gpioSetOutputClient, gpioSetInputClient, gpioSetEncoderCallbackClient, gpioSetPwmFrequencyClient);
}

void MotorControl::pigpioEncoderCountCallback(const hal_pigpio_interfaces::msg::HalPigpioEncoderCount &msg)
{
    if (msg.motor_id == MOTOR_LEFT)
    {
        motorLeft.setEncoderCount(msg.encoder_count);
    }
    else if (msg.motor_id == MOTOR_RIGHT)
    {
        motorRight.setEncoderCount(msg.encoder_count);
    }
    else
    {
        RCLCPP_ERROR(get_logger(), "Encoder count message received for unknown motor!");
    }
    
}

void MotorControl::publishMessage(void)
{
    auto encoderCounts = hal_motor_control_interfaces::msg::HalMotorControl();

    encoderCounts.motor_left_encoder_count = motorLeft.getEncoderCount();
    encoderCounts.motor_right_encoder_count = motorRight.getEncoderCount();
    motorControlPub->publish(encoderCounts);
}

void MotorControl::setPwmLeft(uint16_t dutycycle, bool direction)
{
    motorLeft.setPwmDutyCycleAndDirection(gpioSetPwmDutycycleClient, dutycycle, gpioSetMotorDirectionClient, direction);
}

void MotorControl::setPwmRight(uint16_t dutycycle, bool direction)
{
    motorRight.setPwmDutyCycleAndDirection(gpioSetPwmDutycycleClient, dutycycle, gpioSetMotorDirectionClient, direction);
}