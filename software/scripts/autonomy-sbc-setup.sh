#!/usr/bin/env bash

set -euo pipefail

# exit if run as root
if [ "$EUID" -eq 0 ]; then
  echo "Please run as yourself! Running as superuser (ie, with sudo) breaks the setup."
  exit 1
fi

cd "$(git rev-parse --show-toplevel)"

CONFIG_DIR="$HOME/.config/cyclonedds"
LOCALHOST_XML="$CONFIG_DIR/localhost.xml"
BRIDGE_XML="$CONFIG_DIR/bridge.xml"
ENV_SH="$CONFIG_DIR/env.sh"

SRC_DIR="software/ros_ws/src/autonomy/config"

usage() {
  echo "Usage: $0 [--reverse]"
  echo ""
  echo "Sets up CycloneDDS configuration for SBC topic isolation."
  echo "  --reverse    Remove all created files"
}

reverse() {
  echo "Removing CycloneDDS SBC configuration..."
  rm -fv "$LOCALHOST_XML" "$BRIDGE_XML" "$ENV_SH"
  # Remove directory only if empty
  if rmdir "$CONFIG_DIR" 2>/dev/null; then
    echo "Removed $CONFIG_DIR"
  fi
  echo "Done! Remember to remove 'source ~/.config/cyclonedds/env.sh' from your shell profile."
}

setup() {
  echo "Setting up CycloneDDS SBC configuration..."

  mkdir -p "$CONFIG_DIR"

  cp -v "$SRC_DIR/cyclonedds_localhost.xml" "$LOCALHOST_XML"
  cp -v "$SRC_DIR/cyclonedds_bridge.xml" "$BRIDGE_XML"

  cat >"$ENV_SH" <<'ENVEOF'
# CycloneDDS SBC Topic Isolation
# Source this file in your shell profile to restrict DDS to localhost.
# The topic_republisher process uses a separate bridge config.
export CYCLONEDDS_URI=file://$HOME/.config/cyclonedds/localhost.xml
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENVEOF

  echo ""
  echo "Created:"
  echo "  $LOCALHOST_XML"
  echo "  $BRIDGE_XML"
  echo "  $ENV_SH"
  echo ""
  echo "Add the following to your shell profile (~/.bashrc or ~/.zshrc):"
  echo "  source ~/.config/cyclonedds/env.sh"
}

if [ "${1:-}" = "--reverse" ]; then
  reverse
elif [ "${1:-}" = "--help" ] || [ "${1:-}" = "-h" ]; then
  usage
else
  setup
fi
