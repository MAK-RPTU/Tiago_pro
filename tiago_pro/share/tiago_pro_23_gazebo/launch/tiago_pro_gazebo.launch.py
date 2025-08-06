# Copyright (c) 2024 PAL Robotics S.L. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from dataclasses import dataclass

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_pal.include_utils import include_scoped_launch_py_description
from launch_pal.arg_utils import LaunchArgumentsBase
from launch_pal.robot_arguments import CommonArgs


@dataclass(frozen=True)
class LaunchArguments(LaunchArgumentsBase):
    world_name: DeclareLaunchArgument = CommonArgs.world_name
    navigation: DeclareLaunchArgument = CommonArgs.navigation
    advanced_navigation: DeclareLaunchArgument = CommonArgs.advanced_navigation
    slam: DeclareLaunchArgument = CommonArgs.slam


def declare_actions(launch_description: LaunchDescription, launch_args: LaunchArguments):

    tiago_pro_bringup = include_scoped_launch_py_description(
        pkg_name="tiago_pro_gazebo", paths=["launch", "tiago_pro_gazebo.launch.py"],
        launch_arguments={
            "arm_type_right": "tiago-pro",
            "arm_type_left": "tiago-pro",
            "wrist_model_right": "spherical-wrist",
            "wrist_model_left": "spherical-wrist",
            "tool_changer_right": "True",
            "tool_changer_left": "True",
            "end_effector_right": "pal-pro-gripper",
            "end_effector_left": "pal-pro-gripper",
            "ft_sensor_right": "no-ft-sensor",
            "ft_sensor_left": "no-ft-sensor",
            "laser_model": "sick-571",
            "camera_model": "realsense-d435",
            "base_type": "omni_base",
            "has_laptop_tray": "True",
            "navigation": launch_args.navigation,
            "advanced_navigation": launch_args.advanced_navigation,
            "slam": launch_args.slam,
            "world_name":  launch_args.world_name,
        }
    )

    launch_description.add_action(tiago_pro_bringup)

    return


def generate_launch_description():

    # Create the launch description
    ld = LaunchDescription()

    launch_arguments = LaunchArguments()

    launch_arguments.add_to_launch_description(ld)

    declare_actions(ld, launch_arguments)

    return ld