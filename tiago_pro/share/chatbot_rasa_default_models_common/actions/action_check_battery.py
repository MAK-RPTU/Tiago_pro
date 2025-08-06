# Copyright (c) 2024 PAL Robotics S.L. All rights reserved.
#
# Unauthorized copying of this file, via any medium is strictly prohibited,
# unless it was supplied under the terms of a license agreement or
# nondisclosure agreement with PAL Robotics SL. In this case it may not be
# copied or disclosed except in accordance with the terms of that agreement.

import requests

from chatbot_rasa.types import Action


class ActionCheckBattery(Action):
    @staticmethod
    def name():
        return "action_check_battery"

    def run(self, data):
        get_battery = requests.get("http://control/topic/battery_level").json()
        print(get_battery)
        battery = get_battery["msg"]["data"]
        if battery < 20.0:
            message = _("My battery is low, please put me to charge.")
        else:
            message = _(
                "I have {battery_percent} percent battery.").format(battery_percent=battery)

        res = {"events": [], "responses": [{"text": message}]}
        return res
