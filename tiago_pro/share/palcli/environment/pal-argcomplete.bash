if type register-python-argcomplete3 > /dev/null 2>&1; then
  eval "$(register-python-argcomplete3 pal)"
elif type register-python-argcomplete > /dev/null 2>&1; then
  eval "$(register-python-argcomplete pal)"
fi
