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
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    param_args = [DeclareLaunchArgument(n, default_value=v, description=d) for n, v, d in [
        ('non_verbal_mode', 'false', 'Selects the non-verbal engine instead of the verbal one'),
        ('verbal_engine', 'acapela', 'Engine used if "non_verbal_mode" is false'),
        ('default_locale', 'en_US', 'Default locale used by the verbal engine')]]

    tts_node = LifecycleNode(
        package='tts_engine',
        executable='tts_engine',
        namespace='',
        name='tts_engine',
        parameters=[{a.name: LaunchConfiguration(a.name) for a in param_args}],
        output='both', emulate_tty=True)

    configure_event = EmitEvent(event=ChangeState(
        lifecycle_node_matcher=matches_action(tts_node),
        transition_id=Transition.TRANSITION_CONFIGURE))

    activate_event = RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=tts_node, goal_state='inactive',
        entities=[EmitEvent(event=ChangeState(
            lifecycle_node_matcher=matches_action(tts_node),
            transition_id=Transition.TRANSITION_ACTIVATE))]))

    return LaunchDescription([
        *param_args,
        tts_node,
        configure_event,
        activate_event
    ])


if __name__ == '__main__':
    generate_launch_description()
