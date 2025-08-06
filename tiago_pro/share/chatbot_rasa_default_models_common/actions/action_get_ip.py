# Copyright (c) 2024 PAL Robotics S.L. All rights reserved.
#
# Unauthorized copying of this file, via any medium is strictly prohibited,
# unless it was supplied under the terms of a license agreement or
# nondisclosure agreement with PAL Robotics SL. In this case it may not be
# copied or disclosed except in accordance with the terms of that agreement.

import socket

from chatbot_rasa.types import Action


class ActionGetIp(Action):
    @staticmethod
    def name():
        return "action_get_ip"

    def extract_ip(self):
        st = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            st.connect(("10.255.255.255", 1))
            IP = st.getsockname()[0]
        except Exception:
            IP = "127.0.0.1"
        finally:
            st.close()
        return IP

    def run(self, data):
        res = {
            "events": [],
            "responses": [{"text": _("My internet address is") + " " + self.extract_ip()}],
        }
        return res
