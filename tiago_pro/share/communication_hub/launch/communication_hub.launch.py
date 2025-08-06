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
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_pal import get_pal_configuration
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    activate_arg = DeclareLaunchArgument(
        'activate', default_value='False', description="Activate node on startup")
    chatbot_arg = DeclareLaunchArgument(
        'chatbot', default_value='chatbot', description="Chatbot node name")
    activate = LaunchConfiguration('activate')
    chatbot = LaunchConfiguration('chatbot')

    pkg = 'communication_hub'
    namespace = ''
    node_name = 'communication_hub'
    node_executable = 'communication_hub'

    ld = LaunchDescription()
    config = get_pal_configuration(pkg=pkg, node=node_name, ld=ld)

    node = LifecycleNode(
        package=pkg,
        executable=node_executable,
        namespace=namespace,
        name=node_name,
        parameters=config["parameters"],
        remappings=list(config["remappings"]) + [
            ('chatbot/get_response', [chatbot, TextSubstitution(text='/get_response')]),
            ('chatbot/reset', [chatbot, TextSubstitution(text='/reset')])],
        arguments=config["arguments"],
        output='both', emulate_tty=True)

    configure_event = EmitEvent(event=ChangeState(
        lifecycle_node_matcher=matches_action(node),
        transition_id=Transition.TRANSITION_CONFIGURE))

    activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=node, goal_state='inactive',
            entities=[EmitEvent(event=ChangeState(
                lifecycle_node_matcher=matches_action(node),
                transition_id=Transition.TRANSITION_ACTIVATE))], handle_once=True),
        condition=IfCondition(activate))

    ld.add_action(activate_arg)
    ld.add_action(chatbot_arg)
    ld.add_action(node)
    ld.add_action(configure_event)
    ld.add_action(activate_event)
    return ld
