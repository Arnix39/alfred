# hal_imu package

## Overview

This package contains 3 nodes related to the IMU (**I**nertial **M**easurement **U**nit), a MPU6050 from TDK InvenSense:
- `hal_imuI2cInit_node`: This node handles the opening and closing of the I2C communication with the IMU (using `HalPigpiogI2cOpen` and `HalPigpiogI2cClose` services). It also provides the I2C handle of the IMU through the `HalImuGetHandle` service.
- `hal_imuDmpWritingServer_node`: This node provides the implementation of an action server (`HalImuWriteDmp`) to program the DMP (**D**igital **M**otion **P**rocessor) present on the IMU (using the functions defined in `hal_common` package).
- `hal_imu_node`: This node configures the IMU (including the DMP) and then publishes the IMU data at a 100Hz rate. 

NB: The readings from the IMU are actually performed and published by [`hal_pigpio_node`](../hal_pigpio/README.md) and `hal_imu_node` is only forwarding these messages (the latency introduced by multiple layers of ROS services calls is too high, this is anyway a design improvement point).

## Interfaces

### Topics

The IMU readings are published as [IMU messages](https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/Imu.msg) on `imuData`.

## Dependencies

### Internal

- [`hal_pigpio_interfaces`](../hal_pigpio_interfaces/README.md)
- [`hal_imu_interfaces`](../hal_imu_interfaces/README.md)
- [`hal_common`](../hal_common/README.md)
- [`common_utils`](../../utils/common_utils/README.md)

### External

- `sensor_msgs`
- `ament_cmake`
- `ament_lint_auto`
- `ament_lint_common`
- `ament_cmake_gtest`
- `rclcpp_lifecycle`
- `rclcpp_action`
