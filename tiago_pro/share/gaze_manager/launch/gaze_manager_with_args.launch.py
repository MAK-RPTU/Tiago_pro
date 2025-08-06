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
    visualize_trajectory_arg = DeclareLaunchArgument(
        'visualize_trajectory',
        default_value='False',
        description='Wether or not publishing a MarkerArray object '
                    + 'representing the head trajectory')
    head_aligning_frame_arg = DeclareLaunchArgument(
        'head_aligning_frame',
        default_value='head_front_camera_rgb_frame',
        description='The name of the frame which x axis '
                    + 'has to be aligned with the goal')
    P_arg = DeclareLaunchArgument(
        'P',
        default_value='0.4',
        description='The proportional coefficient '
                    + 'for the PID velocity regulator')
    D_arg = DeclareLaunchArgument(
        'D',
        default_value='0.01',
        description='The derivative coefficient '
                    + 'for the PID velocity regulator')
    error_threshold_arg = DeclareLaunchArgument(
        'error_threshold',
        default_value='0.1',
        description='Maximum accepted error threshold (radians)')
    eyes_available_arg = DeclareLaunchArgument(
        'eyes_available',
        default_value='False',
        description='Whether or not the robot has eyes')

    gaze_manager_node = LifecycleNode(
        package='gaze_manager',
        executable='gaze_manager',
        namespace='',
        name='gaze_manager',
        parameters=[{'visualize_trajectory': LaunchConfiguration('visualize_trajectory'),
                     'head_aligning_frame': LaunchConfiguration('head_aligning_frame'),
                     'P': LaunchConfiguration('P'),
                     'D': LaunchConfiguration('D'),
                     'error_threshold': LaunchConfiguration('error_threshold'),
                     'eyes_available': LaunchConfiguration('eyes_available')}])

    configure_event = EmitEvent(event=ChangeState(
        lifecycle_node_matcher=matches_action(gaze_manager_node),
        transition_id=Transition.TRANSITION_CONFIGURE))

    activate_event = RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=gaze_manager_node, goal_state='inactive',
        entities=[EmitEvent(event=ChangeState(
            lifecycle_node_matcher=matches_action(gaze_manager_node),
            transition_id=Transition.TRANSITION_ACTIVATE))]))

    return LaunchDescription([
       visualize_trajectory_arg,
       head_aligning_frame_arg,
       P_arg,
       D_arg,
       error_threshold_arg,
       eyes_available_arg,
       gaze_manager_node,
       configure_event,
       activate_event])
