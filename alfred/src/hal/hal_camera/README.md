# hal_camera package

## Overview

This package contains the node (`hal_camera_node`) that captures and publishes images using the camera (`Raspberry Pi Camera V2`) at a 5Hz rate. 

## Interfaces

### Topics

The images are published as [image messages](https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/Image.msg) on `cameraImage`.

## Dependencies

### Internal

- [`common_utils`](../../utils/common_utils/README.md)

### External

- `ament_cmake`
- `ament_lint_auto`
- `ament_lint_common`
- `ament_cmake_gtest`
- `rclcpp_lifecycle`
- `cv_bridge`
- `libopencv-dev`
- `sensor_msgs`
- `std_msgs`
