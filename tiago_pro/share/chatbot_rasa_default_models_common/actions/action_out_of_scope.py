# Copyright (c) 2024 PAL Robotics S.L. All rights reserved.
#
# Unauthorized copying of this file, via any medium is strictly prohibited,
# unless it was supplied under the terms of a license agreement or
# nondisclosure agreement with PAL Robotics SL. In this case it may not be
# copied or disclosed except in accordance with the terms of that agreement.

from chatbot_rasa.types import Action

from chatbot_rasa_default_models_common.unknown_intent import unknown_intent_response


class ActionOutOfScope(Action):
    @staticmethod
    def name():
        return "action_out_of_scope"

    def run(self, data):
        return unknown_intent_response(data)
