# Copyright (c) 2024 PAL Robotics S.L. All rights reserved.
#
# Unauthorized copying of this file, via any medium is strictly prohibited,
# unless it was supplied under the terms of a license agreement or
# nondisclosure agreement with PAL Robotics SL. In this case it may not be
# copied or disclosed except in accordance with the terms of that agreement.

import urllib.error
import urllib.request

from chatbot_rasa.types import Action


class ActionCheckInternet(Action):
    @staticmethod
    def name():
        return "action_check_internet"

    def connect(self, host="http://google.com"):
        try:
            urllib.request.urlopen(host)
            return True
        except urllib.error.URLError:
            return False

    def run(self, data):
        if self.connect():
            internet_msgs = _("All good, I have access to internet.")
        else:
            internet_msgs = _("I do not have access to internet at the moment.")
        res = {"events": [], "responses": [{"text": internet_msgs}]}
        return res
