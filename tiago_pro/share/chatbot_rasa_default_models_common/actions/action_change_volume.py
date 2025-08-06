# Copyright (c) 2024 PAL Robotics S.L. All rights reserved.
#
# Unauthorized copying of this file, via any medium is strictly prohibited,
# unless it was supplied under the terms of a license agreement or
# nondisclosure agreement with PAL Robotics SL. In this case it may not be
# copied or disclosed except in accordance with the terms of that agreement.

import random

import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters, GetParameters
from rclpy.parameter import Parameter

from chatbot_rasa.types import Action


class VolumeSetter(Node):
    def __init__(self):
        super().__init__('rasa_action_set_volume')

        self.set_client = self.create_client(
            SetParameters, '/volume/set_parameters')
        self.get_client = self.create_client(
            GetParameters, '/volume/get_parameters')

        while not self.set_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                "Waiting for volume_control_pulseaudio parameters service to"
                " become available...")

    def get_volume(self):
        # Create the request with the list of parameters to set
        request = GetParameters.Request()
        request.names = ['playback']

        # Call the service
        future = self.get_client.call_async(request)

        # Wait for the result
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            return future.result().values[0].integer_value
        else:
            self.get_logger().error('Failed to get volume parameter')
            return None

    def set_volume(self, volume_level):
        # Create a Parameter object
        param = Parameter("playback", Parameter.Type.INTEGER, volume_level)

        # Create the request with the list of parameters to set
        request = SetParameters.Request()
        request.parameters = [param.to_parameter_msg()]

        # Call the service
        future = self.set_client.call_async(request)

        # Wait for the result
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(
                f'Successfully set volume to {volume_level}')
        else:
            self.get_logger().error('Failed to set volume parameter')


class ActionChangeVolume(Action):
    @staticmethod
    def name():
        return "action_change_volume"

    def __init__(self):
        # no need to call rclpy.init() here, as the ROS context
        # is already created by the action server
        self.volume_setter = VolumeSetter()

        self.last_volume = self.volume_setter.get_volume()

    def run(self, data: dict):

        intent = data["tracker"]["latest_message"].get("intent", None)
        if intent:
            intent = intent["name"]

        volume = self.volume_setter.get_volume()

        msg_ok = random.choice([
                _("Sure"),
                _("OK"),
                _("Of course"),
                _("No problem"),
                _("Got it")
                ])

        if intent == "increase_volume":
            volume = self.last_volume + 10
            self.last_volume = volume
            msg = msg_ok
        if intent == "decrease_volume":
            volume = self.last_volume - 10
            self.last_volume = volume
            msg = msg_ok
        if intent == "mute":
            volume = 0
            msg = _("Ok, I mute myself")

        self.volume_setter.set_volume(volume)

        res = {"events": [], "responses": [{"text": msg}]}
        return res


if __name__ == '__main__':

    rclpy.init(args=None)
    action = ActionChangeVolume()
    print(action.run({"hello": "world"}))
    rclpy.shutdown()
