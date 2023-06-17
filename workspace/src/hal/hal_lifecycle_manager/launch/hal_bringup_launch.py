# Copyright (c) 2023 Arnix Robotix
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import launch
import launch.actions
import launch.events

import launch_ros.actions
import launch_ros.events
import launch_ros.events.lifecycle

import lifecycle_msgs.msg


def generate_launch_description():
    hal_lifecycle_manager_node = launch_ros.actions.LifecycleNode(
                name='hal_lifecycle_manager',
                package='hal_lifecycle_manager',
                executable='hal_lifecycle_manager_node',
                namespace='',
                output='screen')

    hal_pigpio_node = launch_ros.actions.LifecycleNode(
                name='hal_pigpio',
                package='hal_pigpio',
                executable='hal_pigpio_node',
                namespace='',
                output='screen')

    hal_camera_node = launch_ros.actions.LifecycleNode(
                name='hal_camera',
                package='hal_camera',
                executable='hal_camera_node',
                namespace='',
                output='screen')

    hal_proxsens_node = launch_ros.actions.LifecycleNode(
                name='hal_proxsens',
                package='hal_proxsens',
                executable='hal_proxsens_node',
                namespace='',
                output='screen')

    hal_imuI2cInit_node = launch_ros.actions.LifecycleNode(
                name='hal_imuI2cInit',
                package='hal_imu',
                executable='hal_imuI2cInit_node',
                namespace='',
                output='screen')

    hal_imuDmpWritingServer_node = launch_ros.actions.LifecycleNode(
                name='hal_imuDmpWritingServer',
                package='hal_imu',
                executable='hal_imuDmpWritingServer_node',
                namespace='',
                output='screen')

    hal_imu_node = launch_ros.actions.LifecycleNode(
                name='hal_imu',
                package='hal_imu',
                executable='hal_imu_node',
                namespace='',
                output='screen')

    hal_motor_control_node = launch_ros.actions.LifecycleNode(
                name='hal_motor_control',
                package='hal_motor_control',
                executable='hal_motor_control_node',
                namespace='',
                output='screen')

    hal_pose_manager_node = launch_ros.actions.LifecycleNode(
                name='hal_pose_manager',
                package='hal_pose_manager',
                executable='hal_pose_manager_node',
                namespace='',
                output='screen')

    load_nodes = launch.actions.GroupAction(
        actions=[
            hal_lifecycle_manager_node,
            hal_pigpio_node,
            hal_camera_node,
            hal_proxsens_node,
            hal_imuI2cInit_node,
            hal_imuDmpWritingServer_node,
            hal_imu_node,
            hal_motor_control_node,
            hal_pose_manager_node,
        ]
    )

    emit_event_to_request_lifecycle_manager_configure_transition = launch.actions.EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(hal_lifecycle_manager_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    register_event_handler_for_lifecycle_manager_reaches_inactive_state = \
        launch.actions.RegisterEventHandler(
            launch_ros.event_handlers.OnStateTransition(
                target_lifecycle_node=hal_lifecycle_manager_node, goal_state='inactive',
                entities=[
                    launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
                        lifecycle_node_matcher=launch.events.matches_action(
                            hal_lifecycle_manager_node),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                    )),
                ],
            )
        )

    register_event_handler_for_lifecycle_manager_reaches_active_state = \
        launch.actions.RegisterEventHandler(
            launch_ros.event_handlers.OnStateTransition(
                target_lifecycle_node=hal_lifecycle_manager_node, goal_state='active',
                entities=[
                    launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
                        lifecycle_node_matcher=launch.events.matches_action(
                            hal_pigpio_node),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
                    )),
                    launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
                        lifecycle_node_matcher=launch.events.matches_action(
                            hal_camera_node),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
                    )),
                    launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
                        lifecycle_node_matcher=launch.events.matches_action(
                            hal_proxsens_node),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
                    )),
                    launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
                        lifecycle_node_matcher=launch.events.matches_action(
                            hal_imuI2cInit_node),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
                    )),
                    launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
                        lifecycle_node_matcher=launch.events.matches_action(
                            hal_imuDmpWritingServer_node),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
                    )),
                    launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
                        lifecycle_node_matcher=launch.events.matches_action(
                            hal_imu_node),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
                    )),
                    launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
                        lifecycle_node_matcher=launch.events.matches_action(
                            hal_motor_control_node),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
                    )),
                    launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
                        lifecycle_node_matcher=launch.events.matches_action(
                            hal_pose_manager_node),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
                    )),
                ],
            )
        )

    register_event_handler_for_hal_camera_reaches_inactive_state = \
        launch.actions.RegisterEventHandler(
            launch_ros.event_handlers.OnStateTransition(
                target_lifecycle_node=hal_camera_node, goal_state='inactive',
                entities=[
                    launch.actions.LogInfo(
                        msg="hal_camera_node inactive, activating."),
                    launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
                        lifecycle_node_matcher=launch.events.matches_action(hal_camera_node),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                    )),
                ],
            )
        )

    register_event_handler_for_hal_pigpio_reaches_inactive_state = \
        launch.actions.RegisterEventHandler(
            launch_ros.event_handlers.OnStateTransition(
                target_lifecycle_node=hal_pigpio_node, goal_state='inactive',
                entities=[
                    launch.actions.LogInfo(
                        msg="hal_pigpio_node inactive, activating."),
                    launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
                        lifecycle_node_matcher=launch.events.matches_action(hal_pigpio_node),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                    )),
                ],
            )
        )

    register_event_handler_for_hal_pigpio_reaches_active_state = \
        launch.actions.RegisterEventHandler(
            launch_ros.event_handlers.OnStateTransition(
                target_lifecycle_node=hal_pigpio_node, goal_state='active',
                entities=[
                    launch.actions.LogInfo(
                        msg="hal_pigpio_node active, activating hal_proxsens_node, " +
                            "hal_imuI2cInit_node and hal_motor_control_node."),
                    launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
                        lifecycle_node_matcher=launch.events.matches_action(hal_proxsens_node),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                    )),
                    launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
                        lifecycle_node_matcher=launch.events.matches_action(hal_imuI2cInit_node),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                    )),
                    launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
                        lifecycle_node_matcher=launch.events.matches_action(
                            hal_motor_control_node),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                    ))
                ],
            )
        )

    register_event_handler_for_hal_imuI2cInit_reaches_active_state = \
        launch.actions.RegisterEventHandler(
            launch_ros.event_handlers.OnStateTransition(
                target_lifecycle_node=hal_imuI2cInit_node, goal_state='active',
                entities=[
                    launch.actions.LogInfo(
                        msg="hal_imuI2cInit_node active, " +
                            "activating hal_imuDmpWritingServer_node."),
                    launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
                        lifecycle_node_matcher=launch.events.matches_action(
                            hal_imuDmpWritingServer_node),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                    )),
                ],
            )
        )

    register_event_handler_for_hal_imuDmpWritingServer_reaches_active_state = \
        launch.actions.RegisterEventHandler(
            launch_ros.event_handlers.OnStateTransition(
                target_lifecycle_node=hal_imuDmpWritingServer_node, goal_state='active',
                entities=[
                    launch.actions.LogInfo(
                        msg="hal_imuDmpWritingServer_node active, activating hal_imu_node."),
                    launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
                        lifecycle_node_matcher=launch.events.matches_action(hal_imu_node),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                    )),
                ],
            )
        )

    register_event_handler_for_hal_imu_reaches_active_state = \
        launch.actions.RegisterEventHandler(
            launch_ros.event_handlers.OnStateTransition(
                target_lifecycle_node=hal_imu_node, goal_state='active',
                entities=[
                    launch.actions.LogInfo(
                        msg="hal_imu_node active, activating hal_pose_manager_node."),
                    launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
                        lifecycle_node_matcher=launch.events.matches_action(hal_pose_manager_node),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                    )),
                ],
            )
        )

    ld = launch.LaunchDescription()
    ld.add_action(load_nodes)
    ld.add_action(register_event_handler_for_lifecycle_manager_reaches_inactive_state)
    ld.add_action(register_event_handler_for_lifecycle_manager_reaches_active_state)
    ld.add_action(register_event_handler_for_hal_camera_reaches_inactive_state)
    ld.add_action(register_event_handler_for_hal_pigpio_reaches_inactive_state)
    ld.add_action(register_event_handler_for_hal_pigpio_reaches_active_state)
    ld.add_action(register_event_handler_for_hal_imuI2cInit_reaches_active_state)
    ld.add_action(register_event_handler_for_hal_imuDmpWritingServer_reaches_active_state)
    ld.add_action(register_event_handler_for_hal_imu_reaches_active_state)

    ld.add_action(emit_event_to_request_lifecycle_manager_configure_transition)

    return ld
