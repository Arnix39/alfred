import launch
import launch.actions
import launch.events

import launch_ros.actions
import launch_ros.events
import launch_ros.events.lifecycle

import lifecycle_msgs.msg

def generate_launch_description():
    ld = launch.LaunchDescription()

    return ld