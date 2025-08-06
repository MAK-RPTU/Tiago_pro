# Copyright (c) 2025 PAL Robotics S.L. All rights reserved.
#
# Unauthorized copying of this file, via any medium is strictly prohibited,
# unless it was supplied under the terms of a license agreement or
# nondisclosure agreement with PAL Robotics SL. In this case it may not be
# copied or disclosed except in accordance with the terms of that agreement.

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    serial_port = LaunchConfiguration('serial_port')
    laser_name = LaunchConfiguration('laser_name')

    serial_port_arg: DeclareLaunchArgument = DeclareLaunchArgument(
        name='serial_port',
        default_value='',
        description='Laser Serial Number. If not set, the first detected laser is used.',
    )

    laser_name_arg: DeclareLaunchArgument = DeclareLaunchArgument(
        name='laser',
        default_value='',
        description='Laser unique name.',
    )

    laser_container = ComposableNodeContainer(
        name='laser_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            # Laser Driver
            ComposableNode(
                package='urg_node',
                plugin='urg_node::UrgNode',
                name=laser_name,
                parameters=[{'serial_port': serial_port}],
            ),
        ],
        output='screen',
    )

    ld = LaunchDescription()
    ld.add_action(serial_port_arg)
    ld.add_action(laser_name_arg)
    ld.add_action(laser_container)
    return ld
