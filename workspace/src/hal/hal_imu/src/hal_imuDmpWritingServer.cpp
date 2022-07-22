#include "hal_imuDmpWritingServer.hpp"

using namespace std::placeholders;

ImuDmpWritingServer::ImuDmpWritingServer() : rclcpp_lifecycle::LifecycleNode("hal_imuDmpWritingServer_node"),
                                             imuHandle(MPU6050_I2C_NO_HANDLE)
{
}

LifecycleCallbackReturn_t ImuDmpWritingServer::on_configure(const rclcpp_lifecycle::State & previous_state)
{
    i2cWriteByteDataClient = this->create_client<hal_pigpio_interfaces::srv::HalPigpioI2cWriteByteData>("hal_pigpioI2cWriteByteData");
    i2cWriteBlockDataClient = this->create_client<hal_pigpio_interfaces::srv::HalPigpioI2cWriteBlockData>("hal_pigpioI2cWriteBlockData");
    imuDmpWritingServer = rclcpp_action::create_server<HalImuWriteDmpAction>(this, 
                                                                             "hal_imuWriteDmp", 
                                                                             std::bind(&ImuDmpWritingServer::handle_goal, this, _1, _2),
                                                                             std::bind(&ImuDmpWritingServer::handle_cancel, this, _1),
                                                                             std::bind(&ImuDmpWritingServer::handle_accepted, this, _1));

    RCLCPP_INFO(get_logger(), "hal_imuDmpWritingServer node configured!");

    return LifecycleCallbackReturn_t::SUCCESS;
}

LifecycleCallbackReturn_t ImuDmpWritingServer::on_activate(const rclcpp_lifecycle::State & previous_state)
{
    RCLCPP_INFO(get_logger(), "hal_imuDmpWritingServer node activated!");

    return LifecycleCallbackReturn_t::SUCCESS;
}

LifecycleCallbackReturn_t ImuDmpWritingServer::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
    RCLCPP_INFO(get_logger(), "hal_imuDmpWritingServer node deactivated!");

    return LifecycleCallbackReturn_t::SUCCESS;
}

LifecycleCallbackReturn_t ImuDmpWritingServer::on_cleanup(const rclcpp_lifecycle::State & previous_state)
{
    RCLCPP_INFO(get_logger(), "hal_imuDmpWritingServer node unconfigured!");

    return LifecycleCallbackReturn_t::SUCCESS;
}

LifecycleCallbackReturn_t ImuDmpWritingServer::on_shutdown(const rclcpp_lifecycle::State & previous_state)
{
    RCLCPP_INFO(get_logger(), "hal_imuDmpWritingServer node shutdown!");

    return LifecycleCallbackReturn_t::SUCCESS;
}

LifecycleCallbackReturn_t ImuDmpWritingServer::on_error(const rclcpp_lifecycle::State & previous_state)
{
    return LifecycleCallbackReturn_t::FAILURE;
}

rclcpp_action::GoalResponse ImuDmpWritingServer::handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const HalImuWriteDmpAction::Goal> goal)
{
    (void)uuid;
    (void)goal;

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ImuDmpWritingServer::handle_cancel(const std::shared_ptr<HalImuWriteDmpGoal> goal_handle)
{
    (void)goal_handle;

    return rclcpp_action::CancelResponse::ACCEPT;
}

void ImuDmpWritingServer::handle_accepted(const std::shared_ptr<HalImuWriteDmpGoal> goal_handle)
{
    std::thread{std::bind(&ImuDmpWritingServer::writeDmp, this, _1), goal_handle}.detach();
}

void ImuDmpWritingServer::writeDmp(const std::shared_ptr<HalImuWriteDmpGoal> goal_handle)
{
    /* This address is the start address of DMP code */
    /* It is coming from InvenSense */
    const uint8_t startAddressMsb = 0x04;
    const uint8_t startAddressLsb = 0x00;

    uint8_t bank = 0;
    uint8_t byteAddressInBank = 0;
    uint8_t chunkAddressInBank = 0;
    uint8_t indexInChunk = 0;
    std::vector<uint8_t> data;
    bool writeSuccess = false;

    auto feedback = std::make_shared<HalImuWriteDmpAction::Feedback>();
    auto result = std::make_shared<HalImuWriteDmpAction::Result>();

    (void)goal_handle;

    RCLCPP_INFO(get_logger(), "Started writing DMP code.");

    for (uint16_t byte = 0; byte < DMP_CODE_SIZE; ++byte)
    {
        indexInChunk = byte % MPU6050_CHUNK_SIZE;
        byteAddressInBank = byte % MPU6050_BANK_SIZE;
        bank = (byte - byteAddressInBank) / MPU6050_BANK_SIZE;

        data.push_back(dmp_memory[byte]);

        /* The chunk is full and ready to be written or we reached the end of the DMP code */
        if ((indexInChunk == (MPU6050_CHUNK_SIZE - 1)) || (byte == DMP_CODE_SIZE - 1))
        {
            chunkAddressInBank = byteAddressInBank - indexInChunk;
            
            if(chunkAddressInBank == 0)
            {
                /* A new bank is starting */
                feedback->bank = bank;
                goal_handle->publish_feedback(feedback);
            }

            if (!writeData(bank, chunkAddressInBank, data))
            {
                RCLCPP_ERROR(get_logger(), "Failed to write DMP code: chunk at address %u of bank %u not written!", chunkAddressInBank, bank);
                goal_handle->abort(result);
                return;
            }

            data.clear();
        }
    }

    writeSuccess = writeByteInRegister(i2cWriteByteDataClient, imuHandle, MPU6050_DMP_START_ADDRESS_H_REGISTER, startAddressMsb);
    writeSuccess &= writeByteInRegister(i2cWriteByteDataClient, imuHandle, MPU6050_DMP_START_ADDRESS_L_REGISTER, startAddressLsb);
    if (!writeSuccess)
    {
        RCLCPP_ERROR(get_logger(), "Failed to write DMP code: start address not written!");
        goal_handle->abort(result);
        return;
    }

    RCLCPP_INFO(get_logger(), "Successfully wrote DMP code.");
    goal_handle->succeed(result);
}

bool ImuDmpWritingServer::writeData(uint8_t bank, uint8_t addressInBank, std::vector<uint8_t> data)
{
    if (addressInBank == 0)
    {
        /* A new bank is starting */
        if (!writeByteInRegister(i2cWriteByteDataClient, imuHandle, MPU6050_BANK_SELECTION_REGISTER, bank))
        {
            return false;
        }
    }

    if (!writeByteInRegister(i2cWriteByteDataClient, imuHandle, MPU6050_ADDRESS_IN_BANK_REGISTER, addressInBank))
    {
        return false;
    }

    if (!writeDataBlock(i2cWriteBlockDataClient, imuHandle, MPU6050_READ_WRITE_REGISTER, data))
    {
        return false;
    }

    return true;
}