# Copyright (c) 2024 PAL Robotics S.L. All rights reserved.
#
# Unauthorized copying of this file, via any medium is strictly prohibited,
# unless it was supplied under the terms of a license agreement or
# nondisclosure agreement with PAL Robotics SL. In this case it may not be
# copied or disclosed except in accordance with the terms of that agreement.

import json

import requests

from chatbot_rasa.types import Action


class ActionGetLanguages(Action):
    @staticmethod
    def name():
        return "action_get_languages"

    def run(self, data):
        from builtins import _

        get_langs = requests.get("http://control/param/language_list").json()
        langs = json.loads(get_langs["value"]["value"])
        print(langs)
        possible_langs = {
            "ca_ES": _("Catalan"),
            "en_GB": _("Catalan"),
            "es_ES": _("Spanish"),
            "en_US": _("English"),
            "fr_FR": _("French"),
            "fr_CA": _("French"),
            "de_DE": _("German"),
            "it_IT": _("Italian"),
        }

        # ensure we do not repeat the same language twice
        langs = set([possible_langs[lang] for lang in langs if lang in possible_langs])

        current_langs = _("I can speak the following languages:") + " "
        current_langs += ", ".join(langs)

        res = {"events": [], "responses": [{"text": current_langs}]}
        return res
