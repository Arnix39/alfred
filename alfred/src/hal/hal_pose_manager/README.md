# hal_pose_manager package

## Overview

This package contains the node responsible for the communication with the controller:
- It computes odometry data from the motors' encoder count values and sends it to the controller. 
- It computes the individual wheel velocity from the velocity commands received from the controller and sends it to the motors.

## Interfaces

### Topics

The odometry information is published in a [`Odometry`](https://github.com/ros2/common_interfaces/blob/humble/nav_msgs/msg/Odometry.msg) message on `odometry`.

The motors' encoders values are received in a [`HalMotorControlEncoders`](../hal_motor_control_interfaces/msg/HalMotorControlEncoders.msg) message on `motorsEncoderCountValue`.

Wheels velocity commands are published in a [`HalMotorControlCommand`](../hal_motor_control_interfaces/msg/HalMotorControlCommand.msg) message on `wheelsVelocityCmd`.

Velocity (linear and angular) commands are received in a [`TwistWithCovariance`](https://github.com/ros2/common_interfaces/blob/humble/geometry_msgs/msg/TwistWithCovariance.msg) message on `cmd_velocity`.

## Dependencies

### Internal

- [`hal_motor_control_interfaces`](../hal_motor_control_interfaces/README.md)
- [`common_utils`](../../utils/common_utils/README.md)

### External
- `std_msgs`
- `geometry_msgs`
- `nav_msgs`
- `ament_cmake`
- `ament_lint_auto`
- `ament_lint_common`
- `ament_cmake_gtest`
- `rclcpp`
- `rclcpp_lifecycle`
