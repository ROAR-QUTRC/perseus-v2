#!/usr/bin/env bash

SESSION="bringup"
LAYOUT=~/perseus-v2/software/zellij-utils/bringup-layout.kdl

# If the session doesn't exist, start it and exit into it
if ! zellij list-sessions | awk '{print $1}' | grep -qx "$SESSION"; then
  echo "No existing session. Creating '$SESSION' with layout..."
  exec zellij --layout "$LAYOUT" --session "$SESSION"
fi

# Otherwise attach to it
echo "Attaching to existing session '$SESSION'"
exec zellij attach "$SESSION"
