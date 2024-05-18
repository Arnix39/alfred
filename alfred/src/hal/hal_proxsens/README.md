# hal_proxsens

## Overview

This package consists of a node (`hal_proxsens_node`) that measures the distance to the nearest obstacle using a proximity sensor (`HC-SR04`). The sensor sends a pulse of 20µs every 100ms and reads an echo that is proportional to the distance (an echo of 59µs corresponds to 1cm). The node then publishes the distance. 

## Interfaces

### Topics

Changes in digital inputs' level are published in a [`HalPigpioEdgeChange`](../hal_pigpio_interfaces/msg/HalPigpioEdgeChange.msg) message on `gpioEdgeChange`.

Raw encoder count values are published in a [`HalPigpioEncoderCount`](../hal_pigpio_interfaces/msg/HalPigpioEncoderCount.msg) message on `hal_pigpioEncoderCount`.

The IMU readings are published as [IMU messages](https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/Imu.msg) on `imuData` at a 100Hz rate.

## Dependencies

### Internal

- [`hal_pigpio_interfaces`](../hal_pigpio_interfaces/README.md)
- [`common_utils`](../../utils/common_utils/README.md)

### External

- `sensor_msgs`
- `std_msgs`
- `ament_cmake`
- `ament_lint_auto`
- `ament_lint_common`
- `ament_cmake_gtest`
- `rclcpp`
- `rclcpp_lifecycle`