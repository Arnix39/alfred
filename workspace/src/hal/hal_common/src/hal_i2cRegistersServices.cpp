#include "hal_i2cRegistersServices.hpp"

int32_t getI2cHandle(imuGetHandleClient_t imuGetHandleClient)
{
  int32_t imuHandle;
  auto imuGetHandleRequest = std::make_shared<hal_imu_interfaces::srv::HalImuGetHandle::Request>();

  auto imuGetHandleCallback = [&imuHandle](ImuGetHandleFuture_t future)
    {
      imuHandle = future.get()->handle;
    };
  auto imuGetHandleFuture = imuGetHandleClient->async_send_request(
    imuGetHandleRequest,
    imuGetHandleCallback);

  return imuHandle;
}

int16_t readByteFromRegister(
  i2cReadByteDataClient_t i2cReadByteDataClient, int32_t imuHandle,
  uint8_t registerToRead)
{
  int16_t byteRead = -1;

  auto i2cReadByteDataRequest =
    std::make_shared<hal_pigpio_interfaces::srv::HalPigpioI2cReadByteData::Request>();

  i2cReadByteDataRequest->handle = imuHandle;
  i2cReadByteDataRequest->device_register = registerToRead;

  auto i2cReadByteDataCallback = [&byteRead](i2cReadByteDataFuture_t future)
    {
      if (future.get()->has_succeeded) {
        byteRead = future.get()->value;
      }
    };
  auto i2cReadByteDataFuture = i2cReadByteDataClient->async_send_request(
    i2cReadByteDataRequest,
    i2cReadByteDataCallback);

  return byteRead;
}

bool writeBitInRegister(
  i2cReadByteDataClient_t i2cReadByteDataClient,
  i2cWriteByteDataClient_t i2cWriteByteDataClient, int32_t imuHandle,
  uint8_t registerToWrite, uint8_t bitToWrite, uint8_t valueOfBit)
{
  uint8_t registerValue;
  int16_t valueRead;
  uint8_t newRegisterValue;

  valueRead = readByteFromRegister(i2cReadByteDataClient, imuHandle, registerToWrite);
  if (valueRead < 0) {
    return false;
  } else {
    registerValue = static_cast<uint8_t>(valueRead);
  }

  if (valueOfBit == 1) {
    newRegisterValue = registerValue | (1 << bitToWrite);
  } else {
    newRegisterValue = registerValue & ~(1 << bitToWrite);
  }

  return writeByteInRegister(i2cWriteByteDataClient, imuHandle, registerToWrite, newRegisterValue);
}

bool writeByteInRegister(
  i2cWriteByteDataClient_t i2cWriteByteDataClient, int32_t imuHandle,
  uint8_t registerToWrite, uint8_t value)
{
  bool writeHasSucceeded = false;

  auto i2cWriteByteDataRequest =
    std::make_shared<hal_pigpio_interfaces::srv::HalPigpioI2cWriteByteData::Request>();

  i2cWriteByteDataRequest->handle = imuHandle;
  i2cWriteByteDataRequest->device_register = registerToWrite;
  i2cWriteByteDataRequest->value = value;

  auto i2cWriteByteDataCallback = [&writeHasSucceeded](i2cWriteByteDataFuture_t future)
    {
      if (future.get()->has_succeeded) {
        writeHasSucceeded = true;
      }
    };
  auto i2cWriteByteDataFuture = i2cWriteByteDataClient->async_send_request(
    i2cWriteByteDataRequest,
    i2cWriteByteDataCallback);

  return writeHasSucceeded;
}

bool writeDataBlock(
  i2cWriteBlockDataClient_t i2cWriteBlockDataClient, int32_t imuHandle,
  uint8_t registerToWrite, std::vector<uint8_t> data)
{
  bool writeHasSucceeded = false;

  auto i2cWriteBlockDataRequest =
    std::make_shared<hal_pigpio_interfaces::srv::HalPigpioI2cWriteBlockData::Request>();

  i2cWriteBlockDataRequest->handle = imuHandle;
  i2cWriteBlockDataRequest->device_register = registerToWrite;
  i2cWriteBlockDataRequest->length = data.size();

  for (uint8_t index = 0; index < data.size(); ++index) {
    i2cWriteBlockDataRequest->data_block.push_back(data.at(index));
  }

  auto i2cWriteBlockDataCallback = [&writeHasSucceeded](i2cWriteBlockDataFuture_t future)
    {
      if (future.get()->has_succeeded) {
        writeHasSucceeded = true;
      }
    };
  auto i2cWriteBlockDataFuture = i2cWriteBlockDataClient->async_send_request(
    i2cWriteBlockDataRequest, i2cWriteBlockDataCallback);

  return writeHasSucceeded;
}
