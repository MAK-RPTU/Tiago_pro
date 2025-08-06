# Copyright (c) 2024 PAL Robotics S.L. All rights reserved.
#
# Unauthorized copying of this file, via any medium is strictly prohibited,
# unless it was supplied under the terms of a license agreement or
# nondisclosure agreement with PAL Robotics SL. In this case it may not be
# copied or disclosed except in accordance with the terms of that agreement.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.substitutions import LaunchConfiguration
from launch.events import matches_action
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    use_saliency_arg = DeclareLaunchArgument(
        'use_saliency',
        default_value='True',
        description='If true, the node can receive priority attention goals on /salient_point')
    eye_frame_arg = DeclareLaunchArgument(
        'eye_frame',
        default_value='sellion_link',
        description='The frame to align with the attention goal')
    base_frame_arg = DeclareLaunchArgument(
        'base_frame',
        default_value='base_link',
        description='The frame used to transform the salient point')
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Whether or not using the simulation time')

    attention_manager_node = LifecycleNode(
        package='attention_manager',
        executable='attention_manager',
        namespace='',
        name='attention_manager',
        parameters=[{'use_saliency': LaunchConfiguration('use_saliency'),
                     'eye_frame': LaunchConfiguration('eye_frame'),
                     'base_frame': LaunchConfiguration('base_frame'),
                     'use_sim_time': LaunchConfiguration('use_sim_time')}])

    configure_event = EmitEvent(event=ChangeState(
        lifecycle_node_matcher=matches_action(attention_manager_node),
        transition_id=Transition.TRANSITION_CONFIGURE))

    activate_event = RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=attention_manager_node, goal_state='inactive',
        entities=[EmitEvent(event=ChangeState(
            lifecycle_node_matcher=matches_action(attention_manager_node),
            transition_id=Transition.TRANSITION_ACTIVATE))]))

    return LaunchDescription([
        use_saliency_arg,
        eye_frame_arg,
        base_frame_arg,
        use_sim_time_arg,
        attention_manager_node,
        configure_event,
        activate_event])
