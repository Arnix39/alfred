# hal_common package

## Overview

This package contains functions related to the I2C communication with the IMU. All these functions are blocking calls (they use synchronized clients, the C++ template `ServiceNodeSync` for such clients is defined in [`common.hpp`](../../utils/common_utils/include/common.hpp)):
- `getI2cHandle`: Returns the IMU's I2C handle provided by the node `hal_imuI2cInit_node` after the communication has been established.
- `readByteFromRegister`: Returns the 1-byte value stored in the given register.
- `readBlockFromRegister`: Returns a  multiple-bytes value stored in the given register.
- `writeBitInRegister`: Write a bit in a given register and returns true if the operation succeeded, false otherwise.
- `writeByteInRegister`: Write a byte in a given register and returns true if the operation succeeded, false otherwise.
- `writeDataBlock`: Write multiple bytes in a given register and returns true if the operation succeeded, false otherwise.

## Interfaces

### Topics

N/A

## Dependencies

### Internal

- [`common_utils`](../../utils/common_utils/README.md)
- [`hal_pigpio_interfaces`](../hal_pigpio_interfaces/README.md)
- [`hal_imu_interfaces`](../hal_imu_interfaces/README.md)
- [`pigpio`](../../hw/pigpio/README.md)

### External

- `ament_cmake`
- `ament_lint_auto`
- `ament_lint_common`
- `ament_cmake_gtest`
- `rclcpp_lifecycle`
