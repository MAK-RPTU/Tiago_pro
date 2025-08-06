# Copyright (c) 2024 PAL Robotics S.L. All rights reserved.
#
# Unauthorized copying of this file, via any medium is strictly prohibited,
# unless it was supplied under the terms of a license agreement or
# nondisclosure agreement with PAL Robotics SL. In this case it may not be
# copied or disclosed except in accordance with the terms of that agreement.

from datetime import datetime

from chatbot_rasa.types import Action


class ActionGetTime(Action):
    @staticmethod
    def name():
        return "action_get_time"

    def run(self, data):
        now = datetime.now()
        print("getting time")
        current_time = now.strftime("%H:%M:%S")
        current_time = current_time.split(":")
        print(current_time[0].lstrip("0"))

        res = {
            "events": [],
            "responses": [
                {
                    "text": _("It is {hours} {minutes}").format(
                        hours=current_time[0].lstrip("0"),
                        minutes=current_time[1].lstrip("0"),
                    )
                }
            ],
        }
        return res
