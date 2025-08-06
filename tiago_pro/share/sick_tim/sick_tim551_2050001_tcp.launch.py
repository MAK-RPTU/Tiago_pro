# Copyright (c) 2020, Stratom Inc.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():
    # launch parameters
    namespace = LaunchConfiguration('namespace')
    autostart = LaunchConfiguration('autostart')

    declare_namespace = DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Top-level namespace')

    declare_autostart = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the app')

    config = os.path.join(
        get_package_share_directory('sick_tim'),
        'cfg',
        'sick_tcp.yaml'
    )

    robot_description_xacro = os.path.join(
        get_package_share_directory('sick_tim'),
        'urdf',
        'example.urdf.xacro'
    )
    print(config)
    print(robot_description_xacro)

    # Nodes launching commands
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=namespace,
        output='screen',
        emulate_tty=True,  # https://github.com/ros2/launch/issues/188
        parameters=[{'robot_description': Command(['xacro', ' ', robot_description_xacro])}]
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_dlo',
        namespace=namespace,
        output='screen',
        emulate_tty=True,  # https://github.com/ros2/launch/issues/188
        parameters=[{'autostart': autostart},
                    {'bond_timeout': 0.0},
                    {'node_names': ['sick_tim_driver']}])

    sick_tim_driver = Node(
        package='sick_tim',
        executable='sick_tim551_2050001',
        name='sick_tim_driver',
        namespace=namespace,
        output='screen',
        emulate_tty=True,  # https://github.com/ros2/launch/issues/188
        parameters=[config])

    # description
    ld = LaunchDescription()

    ld.add_action(declare_namespace)
    ld.add_action(declare_autostart)

    ld.add_action(rsp)
    ld.add_action(lifecycle_manager)
    ld.add_action(sick_tim_driver)

    return ld
