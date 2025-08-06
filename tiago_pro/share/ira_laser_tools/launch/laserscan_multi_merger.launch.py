#!/usr/bin/env python3

# Copyright (c) 2023 PAL Robotics S.L. All rights reserved.
#
# Unauthorized copying of this file, via any medium is strictly prohibited,
# unless it was supplied under the terms of a license agreement or
# nondisclosure agreement with PAL Robotics SL. In this case it may not be
# copied or disclosed except in accordance with the terms of that agreement.
#
# Author: Jose Carlos Garcia

from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler
from launch.events import matches_action
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
from lifecycle_msgs.msg import Transition

from launch_pal import get_pal_configuration


def generate_launch_description():
    # Nodes
    ld = LaunchDescription()
    pkg = 'ira_laser_tools'
    name = 'laserscan_multi_merger'
    config = get_pal_configuration(pkg=pkg, node=name, ld=ld)

    laserscan_multi_merger_node = LifecycleNode(
        package=pkg,
        executable='laserscan_multi_merger_node',
        name=name,
        namespace='',
        output='screen',
        emulate_tty=True,  # https://github.com/ros2/launch/issues/188
        parameters=config['parameters'],
        remappings=config['remappings'],
        arguments=config['arguments'],
    )

    # Configure and Activate
    configure_event = EmitEvent(event=ChangeState(
        lifecycle_node_matcher=matches_action(laserscan_multi_merger_node),
        transition_id=Transition.TRANSITION_CONFIGURE))

    activate_event = RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=laserscan_multi_merger_node, goal_state='inactive',
        entities=[EmitEvent(event=ChangeState(
            lifecycle_node_matcher=matches_action(laserscan_multi_merger_node),
            transition_id=Transition.TRANSITION_ACTIVATE))], handle_once=True))

    # description
    ld.add_action(laserscan_multi_merger_node)
    ld.add_action(configure_event)
    ld.add_action(activate_event)

    return ld
