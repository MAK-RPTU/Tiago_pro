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


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument

from launch_pal.include_utils import include_scoped_launch_py_description
from launch_pal.arg_utils import LaunchArgumentsBase
from launch_pal.robot_arguments import CommonArgs
from tiago_pro_description.launch_arguments import TiagoProArgs

from dataclasses import dataclass


@dataclass(frozen=True)
class LaunchArguments(LaunchArgumentsBase):
    base_type: DeclareLaunchArgument = TiagoProArgs.base_type
    arm_type_right: DeclareLaunchArgument = TiagoProArgs.arm_type_right
    arm_type_left: DeclareLaunchArgument = TiagoProArgs.arm_type_left
    end_effector_right: DeclareLaunchArgument = TiagoProArgs.end_effector_right
    end_effector_left: DeclareLaunchArgument = TiagoProArgs.end_effector_left
    ft_sensor_right: DeclareLaunchArgument = TiagoProArgs.ft_sensor_right
    ft_sensor_left: DeclareLaunchArgument = TiagoProArgs.ft_sensor_left
    wrist_model_right: DeclareLaunchArgument = TiagoProArgs.wrist_model_right
    wrist_model_left: DeclareLaunchArgument = TiagoProArgs.wrist_model_left
    tool_changer_right: DeclareLaunchArgument = TiagoProArgs.tool_changer_right
    tool_changer_left: DeclareLaunchArgument = TiagoProArgs.tool_changer_left
    camera_model: DeclareLaunchArgument = TiagoProArgs.camera_model
    laser_model: DeclareLaunchArgument = TiagoProArgs.laser_model
    use_sim_time: DeclareLaunchArgument = CommonArgs.use_sim_time
    namespace: DeclareLaunchArgument = CommonArgs.namespace
    is_public_sim: DeclareLaunchArgument = CommonArgs.is_public_sim


def declare_actions(launch_description: LaunchDescription, launch_args: LaunchArguments):
    default_controllers = include_scoped_launch_py_description(
        pkg_name='tiago_pro_controller_configuration',
        paths=['launch', 'default_controllers.launch.py'],
        launch_arguments={"arm_type_right": launch_args.arm_type_right,
                          "arm_type_left": launch_args.arm_type_left,
                          "end_effector_right": launch_args.end_effector_right,
                          "end_effector_left": launch_args.end_effector_left,
                          "ft_sensor_right": launch_args.ft_sensor_right,
                          "ft_sensor_left": launch_args.ft_sensor_left,
                          "use_sim_time": launch_args.use_sim_time,
                          "is_public_sim": launch_args.is_public_sim
                          })

    launch_description.add_action(default_controllers)

    play_motion2 = include_scoped_launch_py_description(
        pkg_name='tiago_pro_bringup',
        paths=['launch', 'tiago_pro_play_motion2.launch.py'],
        launch_arguments={"arm_type_right": launch_args.arm_type_right,
                          "arm_type_left": launch_args.arm_type_left,
                          "end_effector_right": launch_args.end_effector_right,
                          "end_effector_left": launch_args.end_effector_left,
                          "wrist_model_right": launch_args.wrist_model_right,
                          "wrist_model_left": launch_args.wrist_model_left,
                          "use_sim_time": launch_args.use_sim_time})

    launch_description.add_action(play_motion2)

    twist_mux = include_scoped_launch_py_description(
        pkg_name="tiago_pro_bringup",
        paths=["launch", "twist_mux.launch.py"],
        launch_arguments={"use_sim_time": launch_args.use_sim_time}
    )

    launch_description.add_action(twist_mux)

    robot_state_publisher = include_scoped_launch_py_description(
        pkg_name='tiago_pro_description',
        paths=['launch', 'robot_state_publisher.launch.py'],
        launch_arguments={"arm_type_right": launch_args.arm_type_right,
                          "arm_type_left": launch_args.arm_type_left,
                          "end_effector_right": launch_args.end_effector_right,
                          "end_effector_left": launch_args.end_effector_left,
                          "ft_sensor_right": launch_args.ft_sensor_right,
                          "ft_sensor_left": launch_args.ft_sensor_left,
                          "wrist_model_right": launch_args.wrist_model_right,
                          "wrist_model_left": launch_args.wrist_model_left,
                          "tool_changer_right": launch_args.tool_changer_right,
                          "tool_changer_left": launch_args.tool_changer_left,
                          "laser_model": launch_args.laser_model,
                          "camera_model": launch_args.camera_model,
                          "base_type": launch_args.base_type,
                          "namespace": launch_args.namespace,
                          "use_sim_time": launch_args.use_sim_time,
                          "is_public_sim": launch_args.is_public_sim
                          })

    launch_description.add_action(robot_state_publisher)

    return


def generate_launch_description():

    # Create the launch description
    ld = LaunchDescription()

    launch_arguments = LaunchArguments()

    launch_arguments.add_to_launch_description(ld)

    declare_actions(ld, launch_arguments)

    return ld
