# Copyright (c) 2024 PAL Robotics S.L. All rights reserved.
#
# Unauthorized copying of this file, via any medium is strictly prohibited,
# unless it was supplied under the terms of a license agreement or
# nondisclosure agreement with PAL Robotics SL. In this case it may not be
# copied or disclosed except in accordance with the terms of that agreement.

import requests

from chatbot_rasa.types import Action


class ActionIsCharging(Action):
    @staticmethod
    def name():
        return "action_is_charging"

    def run(self, data):
        get_charging_status = requests.get("http://control/topic/is_charging").json()
        print(get_charging_status)
        status = get_charging_status["msg"]["data"]
        if status is False:
            message = _("I am not currently charging")
        else:
            message = _("I am currently charging")

        res = {"events": [], "responses": [{"text": message}]}
        return res
