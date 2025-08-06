# Copyright (c) 2024 PAL Robotics S.L. All rights reserved.
#
# Unauthorized copying of this file, via any medium is strictly prohibited,
# unless it was supplied under the terms of a license agreement or
# nondisclosure agreement with PAL Robotics SL. In this case it may not be
# copied or disclosed except in accordance with the terms of that agreement.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    param_args = [DeclareLaunchArgument(n, default_value=v, description=d) for n, v, d in [
        ('default_system_locale', 'en_US', "Default system locale, set on activate transition"),
        ('localized_nodes', "['/tts_engine']",
            "List of nodes which require the locale language to be set")]]

    i18n_manager_node = LifecycleNode(
        package='i18n_manager', executable='i18n_manager', namespace='',
        name='i18n_manager',
        parameters=[{a.name: LaunchConfiguration(a.name) for a in param_args}],
        output='both', emulate_tty=True)

    configure_event = EmitEvent(event=ChangeState(
        lifecycle_node_matcher=matches_action(i18n_manager_node),
        transition_id=Transition.TRANSITION_CONFIGURE))

    activate_event = RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=i18n_manager_node, goal_state='inactive',
        entities=[EmitEvent(event=ChangeState(
            lifecycle_node_matcher=matches_action(i18n_manager_node),
            transition_id=Transition.TRANSITION_ACTIVATE))]))

    return LaunchDescription([
        *param_args,
        i18n_manager_node,
        configure_event,
        activate_event,
    ])
