# Copyright (c) 2024 PAL Robotics S.L. All rights reserved.
#
# Unauthorized copying of this file, via any medium is strictly prohibited,
# unless it was supplied under the terms of a license agreement or
# nondisclosure agreement with PAL Robotics SL. In this case it may not be
# copied or disclosed except in accordance with the terms of that agreement.

from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler, Shutdown
from launch.events import matches_action
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
from lifecycle_msgs.msg import Transition
from launch_pal import get_pal_configuration


def generate_launch_description():

    pkg = 'attention_manager'
    node = 'attention_manager'
    ld = LaunchDescription()

    config = get_pal_configuration(pkg=pkg, node=node, ld=ld)

    attention_manager_node = LifecycleNode(
        package=pkg,
        executable='attention_manager',
        namespace='',
        name=node,
        parameters=config["parameters"],
        remappings=config["remappings"],
        arguments=config["arguments"],
        on_exit=Shutdown()
    )

    configure_event = EmitEvent(event=ChangeState(
        lifecycle_node_matcher=matches_action(attention_manager_node),
        transition_id=Transition.TRANSITION_CONFIGURE))

    activate_event = RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=attention_manager_node, goal_state='inactive',
        entities=[EmitEvent(event=ChangeState(
            lifecycle_node_matcher=matches_action(attention_manager_node),
            transition_id=Transition.TRANSITION_ACTIVATE))]))

    ld.add_action(attention_manager_node)
    ld.add_action(configure_event)
    ld.add_action(activate_event)

    return ld
