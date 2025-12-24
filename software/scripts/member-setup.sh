#!/usr/bin/env bash

# If you're changing this file, make sure to read the systems/software/scripts docs

set -euo pipefail

# exit if run as root
if [ "$EUID" -eq 0 ]; then
  echo "Please run as yourself! Running as superuser (ie, with sudo) breaks the setup."
  exit
fi

# Update and install required packages
sudo apt-get update >/dev/null 2>&1
sudo apt-get install -y gh git direnv >/dev/null 2>&1

# Sign into gh CLI
if ! gh auth status >/dev/null 2>&1; then
  echo "You need to sign into github CLI. Follow these instructions:"
  gh auth login -w -p https
else
  echo "GitHub CLI already logged in."
fi

# Clone the perseus-v2 repo
cd ~
if ! [ -d "perseus-v2" ]; then
  echo "Perseus repo not detected. Cloning now."
  gh repo clone ROAR-QUTRC/perseus-v2
else
  echo "Perseus repo already cloned. Continuing."
fi

# Run the nix-setup script
echo "Running nix-setup.sh script. If asked, accept all config options by typing 'y', then press enter."

cd ~/perseus-v2
./software/scripts/nix-setup.sh

# Check the script ran correctly and didn't fail
EXIT_STATUS=$?

if ! [ $EXIT_STATUS -eq 0 ]; then
  echo "nix-setup.sh script failed (Exit code $EXIT_STATUS). Exiting"
  exit
else
  echo "nix-setup.sh script ran successfully. Continuing"
fi

# Build nix packages
echo "Building nix packages. If asked, accept all config options with 'y'"
/nix/var/nix/profiles/default/bin/nix build --accept-flake-config

echo "Setup script ran successfully!"
echo "Restarting shell"

exec $SHELL
