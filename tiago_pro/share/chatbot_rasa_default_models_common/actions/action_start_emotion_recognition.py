# Copyright (c) 2024 PAL Robotics S.L. All rights reserved.
#
# Unauthorized copying of this file, via any medium is strictly prohibited,
# unless it was supplied under the terms of a license agreement or
# nondisclosure agreement with PAL Robotics SL. In this case it may not be
# copied or disclosed except in accordance with the terms of that agreement.

from __future__ import annotations
import json

from hri_actions_msgs.msg import Intent
from rclpy.serialization import serialize_message

from chatbot_rasa.types import Action


class ActionPresentContent(Action):
    @staticmethod
    def name():
        return "action_start_emotion_recognition"

    def run(self, data: dict):
        intent_msg = Intent()
        intent_msg.intent = intent_msg.START_ACTIVITY
        intent_msg.source = intent_msg.UNKNOWN_AGENT
        intent_msg.modality = intent_msg.MODALITY_SPEECH
        intent_msg.priority = 128
        intent_msg.confidence = 1.0
        intent_msg.data = json.dumps({"object": "emotion_recognition"})

        # append to existing intents
        intents = data["tracker"]["slots"].get("intents", list[str]())
        intents.append(serialize_message(intent_msg).hex())

        return {
            "events": [
                {
                    "event": "slot",
                    "name": "intents",
                    "value": intents,
                }
            ],
            "responses": [{"text": ""}]
        }
