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
from launch_pal import get_pal_configuration
from launch_ros.actions import LifecycleNode


def generate_launch_description():
    pkg = 'audio_capture'
    node = 'audio_capture'
    ld = LaunchDescription()
    config = get_pal_configuration(pkg=pkg, node=node, ld=ld)

    communication_hub_node = LifecycleNode(
        package=pkg,
        executable='audio_capture_node',
        namespace='',
        name=node,
        parameters=config["parameters"],
        remappings=config["remappings"],
        arguments=config["arguments"],
        output='both', emulate_tty=True)

    ld.add_action(communication_hub_node)
    return ld
