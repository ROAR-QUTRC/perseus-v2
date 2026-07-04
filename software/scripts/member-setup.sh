#!/usr/bin/env bash

# If you're changing this file, make sure to read the systems/software/scripts docs

set -euo pipefail

# exit if run as root
if [ "$EUID" -eq 0 ]; then
  echo "Please run as yourself! Running as superuser (ie, with sudo) breaks the setup."
  exit 1
fi

# Update and install required packages
sudo apt-get update >/dev/null 2>&1
sudo apt-get install -y gh git direnv >/dev/null 2>&1

# Sign into gh CLI
if ! gh auth status >/dev/null 2>&1; then
  echo "You need to sign into github CLI. Follow these instructions:"
  gh auth login -w
else
  echo "GitHub CLI already logged in."
fi

# Clone the perseus-lite repo
cd ~
if ! [ -d "perseus-lite" ]; then
  echo "perseus-lite repo not detected. Cloning now."
  gh repo clone DingoOz/perseus-lite
else
  echo "perseus-lite repo already cloned. Continuing."
fi

cd ~/perseus-lite

# Install Pixi if it isn't already on PATH
if ! command -v pixi >/dev/null 2>&1; then
  echo "Pixi not found. Installing now."
  curl -fsSL https://pixi.sh/install.sh | bash
  # The installer adds pixi to ~/.bashrc/~/.zshrc for future shells; pick it
  # up in this one too.
  export PATH="$HOME/.pixi/bin:$PATH"
else
  echo "Pixi already installed. Continuing."
fi

# Resolve + fetch the default environment and build the workspace
echo "Running 'pixi install' (this can take a while on first run)."
pixi install

echo "Building the default workspace with 'pixi run -e default build'."
pixi run -e default build

echo "Setup script ran successfully!"
echo "Restarting shell"

exec "$SHELL"
