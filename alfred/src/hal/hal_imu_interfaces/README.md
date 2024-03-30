# hal_imu_interfaces package

## Overview

This interface package defines:
- The following action server:
    - `HalImuWriteDmp` that programs the DMP (**D**igital **M**otion **P**rocessor) present on the IMU.

- The following service:
    - `HalImuGetHandle` that responds with the IMU's I2C handle.

## Interfaces

### Topics

N/A

## Dependencies

### Internal

N/A

### External

- `action_msgs`
- `ament_cmake`
- `rclcpp`
- `rosidl_default_generators`
- `rosidl_default_runtime`
- `rosidl_interface_packages`
