# Copyright (c) 2025 PAL Robotics S.L. All rights reserved.
#
# Unauthorized copying of this file, via any medium is strictly prohibited,
# unless it was supplied under the terms of a license agreement or
# nondisclosure agreement with PAL Robotics SL. In this case it may not be
# copied or disclosed except in accordance with the terms of that agreement.

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, EmitEvent
from launch.events import matches_action
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
from lifecycle_msgs.msg import Transition


def generate_launch_description():

    serial_number = LaunchConfiguration('serial_number')
    laser_name = LaunchConfiguration('laser_name')

    serial_number_arg: DeclareLaunchArgument = DeclareLaunchArgument(
        name='serial_number',
        default_value='',
        description='Laser Serial Number. If not set, the first detected laser is used.',
    )

    laser_name_arg: DeclareLaunchArgument = DeclareLaunchArgument(
        name='laser_name',
        default_value='sick_driver',
        description='Laser unique name.',
    )

    sick_driver_node = LifecycleNode(
        package='sick_tim',
        executable='sick_tim551_2050001',
        name=laser_name,
        namespace='',
        output='screen',
        emulate_tty=True,  # https://github.com/ros2/launch/issues/188
        parameters=[{'serial_number': serial_number}],
    )

    # Configure and Activate
    configure_event = EmitEvent(event=ChangeState(
        lifecycle_node_matcher=matches_action(sick_driver_node),
        transition_id=Transition.TRANSITION_CONFIGURE))

    activate_event = RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=sick_driver_node, goal_state='inactive',
        entities=[EmitEvent(event=ChangeState(
            lifecycle_node_matcher=matches_action(sick_driver_node),
            transition_id=Transition.TRANSITION_ACTIVATE))], handle_once=True))

    ld = LaunchDescription()
    ld.add_action(serial_number_arg)
    ld.add_action(laser_name_arg)
    ld.add_action(sick_driver_node)
    ld.add_action(configure_event)
    ld.add_action(activate_event)
    return ld
