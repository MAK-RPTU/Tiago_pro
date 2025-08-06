if (( ! ${+_comps} )); then
  autoload -U +X compinit && compinit
fi
autoload -U +X bashcompinit && bashcompinit

# Get this scripts directory
__palcli_completion_dir=${0:a:h}
# Just source the bash version, it works in zsh too
source "$__palcli_completion_dir/pal-argcomplete.bash"
# Cleanup
unset __palcli_completion_dir
