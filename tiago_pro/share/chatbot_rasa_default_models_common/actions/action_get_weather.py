# Copyright (c) 2024 PAL Robotics S.L. All rights reserved.
#
# Unauthorized copying of this file, via any medium is strictly prohibited,
# unless it was supplied under the terms of a license agreement or
# nondisclosure agreement with PAL Robotics SL. In this case it may not be
# copied or disclosed except in accordance with the terms of that agreement.

import json

import requests

from chatbot_rasa.types import Action


class ActionGetWeather(Action):
    @staticmethod
    def name():
        return "action_get_weather"

    def run(self, data):
        loc = json.loads(data["tracker"]["slots"]["location"])
        api_key = "66230fca5a599eccdbdbdd4ee2fb798c"

        try:
            current = requests.get(
                "http://api.openweathermap.org/data/2.5/weather"
                "?q={}&appid={}&lang=en&units=metric".format(loc, api_key)
            ).json()
        except requests.exceptions.ConnectionError:
            res = {
                "events": [],
                "responses": [
                    {
                        "text": _(
                            "Sorry, I can not tell you about the weather right now, "
                            "as I can not connect to the weather server"
                        )
                    }
                ],
            }
            return res

        country = current["sys"]["country"]  # noqa: F841
        city = current["name"]
        condition = current["weather"][0]["description"]
        temperature_c = int(current["main"]["temp"])
        humidity = current["main"]["humidity"]  # noqa: F841
        wind_mph = current["wind"]["speed"]  # noqa: F841
        response = _(
            "It is currently {condition} in {city}. The temperature is {temperature_c} degrees."
        ).format(condition=condition, city=city, temperature_c=temperature_c)
        res = {"events": [], "responses": [{"text": response}]}

        return res
