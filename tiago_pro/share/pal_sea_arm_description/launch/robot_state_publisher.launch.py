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


import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_param_builder import load_xacro
from launch_pal.arg_utils import read_launch_argument
from launch_pal.arg_utils import LaunchArgumentsBase
from launch_pal.robot_arguments import CommonArgs
from dataclasses import dataclass

from pal_sea_arm_description.launch_arguments import SEAArmArgs


@dataclass(frozen=True)
class LaunchArguments(LaunchArgumentsBase):

    end_effector: DeclareLaunchArgument = SEAArmArgs.end_effector
    ft_sensor: DeclareLaunchArgument = SEAArmArgs.ft_sensor

    # For future changes in the wrist
    wrist_model: DeclareLaunchArgument = SEAArmArgs.wrist_model
    tool_changer: DeclareLaunchArgument = SEAArmArgs.tool_changer
    arm_type: DeclareLaunchArgument = SEAArmArgs.arm_type

    use_sim_time: DeclareLaunchArgument = CommonArgs.use_sim_time
    namespace: DeclareLaunchArgument = CommonArgs.namespace


def declare_actions(launch_description: LaunchDescription, launch_args: LaunchArguments):

    launch_description.add_action(OpaqueFunction(
        function=create_robot_description_param))

    rsp = Node(package='robot_state_publisher',
               executable='robot_state_publisher',
               output='both',
               parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time'),
                            'robot_description': LaunchConfiguration('robot_description')}])

    launch_description.add_action(rsp)
    return


def create_robot_description_param(context, *args, **kwargs):

    xacro_file_path = Path(os.path.join(
        get_package_share_directory('pal_sea_arm_description'),
        'robots', 'pal_sea_arm.urdf.xacro'))

    xacro_input_args = {
        'end_effector': read_launch_argument('end_effector', context),
        'wrist_model': read_launch_argument('wrist_model', context),
        'ft_sensor': read_launch_argument('ft_sensor', context),
        'arm_type': read_launch_argument('arm_type', context),
        'tool_changer': read_launch_argument('tool_changer', context),
        'use_sim': read_launch_argument('use_sim_time', context),
        'namespace': read_launch_argument('namespace', context),
    }
    robot_description = load_xacro(xacro_file_path, xacro_input_args)

    return [SetLaunchConfiguration('robot_description', robot_description)]


def generate_launch_description():

    # Create the launch description
    ld = LaunchDescription()

    launch_arguments = LaunchArguments()

    launch_arguments.add_to_launch_description(ld)

    declare_actions(ld, launch_arguments)

    return ld
