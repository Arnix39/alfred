import launch
import launch.actions
import launch.events

import launch_ros.actions
import launch_ros.events
import launch_ros.events.lifecycle

import lifecycle_msgs.msg

def generate_launch_description():
    ld = launch.LaunchDescription()

    hal_pigpio_node = launch_ros.actions.LifecycleNode(
        name='hal_pigpio', namespace='',
        package='hal_pigpio', executable='hal_pigpio_node', output='screen')

    hal_proxsens_node = launch_ros.actions.LifecycleNode(
        name='hal_proxsens', namespace='',
        package='hal_proxsens', executable='hal_proxsens_node', output='screen')

    register_event_handler_for_hal_pigpio_reaches_inactive_state = launch.actions.RegisterEventHandler(
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

    register_event_handler_for_hal_pigpio_reaches_active_state = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=hal_pigpio_node, goal_state='active',
            entities=[
                launch.actions.LogInfo(
                    msg="hal_pigpio_node active, activating hal_proxsens_node."),
                launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
                    lifecycle_node_matcher=launch.events.matches_action(hal_proxsens_node),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                )),
            ],
        )
    )

    emit_event_to_request_hal_pigpio_configure_transition = launch.actions.EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(hal_pigpio_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    emit_event_to_request_hal_proxsens_configure_transition = launch.actions.EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(hal_proxsens_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    ld.add_action(register_event_handler_for_hal_pigpio_reaches_inactive_state)
    ld.add_action(register_event_handler_for_hal_pigpio_reaches_active_state)
    ld.add_action(hal_pigpio_node)
    ld.add_action(hal_proxsens_node)
    ld.add_action(emit_event_to_request_hal_pigpio_configure_transition)
    ld.add_action(emit_event_to_request_hal_proxsens_configure_transition)

    return ld