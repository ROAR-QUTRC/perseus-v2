#! /usr/bin/env bash

set -euo pipefail

# exit if run as root
if [ "$EUID" -eq 0 ]; then
  echo "Please run as yourself! Running as superuser (ie, with sudo) breaks the setup."
  exit
fi

echo "Setting up home-manager"
# will back up pre-existing files to filename.bak if needed
# Allows changes to apply even on manually configured systems
nix run home-manager/master -- switch --flake . -b bak

echo "Changing the shell requires sudo - please enter your password if prompted."
sudo usermod -s "$HOME/.nix-profile/bin/zsh" "$(whoami)"

echo "Please restart your shell for the changes to take effect."
