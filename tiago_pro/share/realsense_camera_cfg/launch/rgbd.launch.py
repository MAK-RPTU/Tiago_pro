# Copyright (c) 2025 PAL Robotics S.L. All rights reserved.
#
# Unauthorized copying of this file, via any medium is strictly prohibited,
# unless it was supplied under the terms of a license agreement or
# nondisclosure agreement with PAL Robotics SL. In this case it may not be
# copied or disclosed except in accordance with the terms of that agreement.

import rclpy
from rclpy.node import Node as RclpyNode
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode

from launch_pal import get_pal_configuration


def generate_launch_description():

    ld = LaunchDescription()
    rgbd_camera_node = 'rgbd_camera'
    rgbd_camera_config = get_pal_configuration(
        pkg='realsense_camera_cfg',
        node=rgbd_camera_node,
        ld=ld,
        cmdline_args=True
    )

    # If the container node already exists, just load the component
    rclpy.init()
    node = RclpyNode('node_checker')
    rclpy.spin_once(node, timeout_sec=1.0)
    if 'rgbd_container' not in node.get_node_names():
        rgbd_container = Node(
            name='rgbd_container',
            package='rclcpp_components',
            executable='component_container',
            emulate_tty=True,
            output='screen',
        )
        ld.add_action(rgbd_container)
    rclpy.shutdown()

    realsense_component = LoadComposableNodes(
        target_container='rgbd_container',
        composable_node_descriptions=[
            ComposableNode(
                package='realsense2_camera',
                plugin='realsense2_camera::RealSenseNodeFactory',
                name=rgbd_camera_node,
                namespace=LaunchConfiguration('camera_name'),
                parameters=rgbd_camera_config['parameters'],
                remappings=rgbd_camera_config['remappings'],
            ),
        ],
    )

    ld.add_action(realsense_component)
    return ld
