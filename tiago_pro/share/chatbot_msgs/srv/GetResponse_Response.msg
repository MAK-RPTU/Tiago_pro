
# The text of the response to the request.
# It might be left empty if no suitable response can be generated in the given context.
# E.g., if an intent was detected, the response might depend on how the intent is handled by the system.
# In this case, a verbalized response might be solicited with a second request, using the intent processing result as system input.
string response

# The list of intents that were detected in the request or generated in response.
# The intents are in order of relevance, with the most relevant one first.
# It might be left empty if no intent is detected.
hri_actions_msgs/Intent[] intents

# The error message.
# This is not empty only if the request could not be fully processed.
string error
