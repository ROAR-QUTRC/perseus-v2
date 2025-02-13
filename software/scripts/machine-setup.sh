#! /usr/bin/env bash

set -euo pipefail

# exit if run as root
if [ "$EUID" -eq 0 ]; then
  echo "Please run as yourself! Running as superuser (ie, with sudo) breaks the setup."
  exit
fi

# cd to the scripts directory
cd "$(git rev-parse --show-toplevel)"/software/scripts

# only run the setup from remote flake if home-manager isn't already installed
if ! command -v home-manager &>/dev/null; then
  echo "Setting up home manager..."
  # will back up pre-existing files to filename.bak if needed
  # Allows changes to apply even on systems with pre-existing config
  # which home-manager will take over from
  nix run home-manager/master -- switch --flake . -b bak
else
  echo "home manager already present on system!"
fi

SHELL_PATH="$HOME/.nix-profile/bin/zsh"

echo "The next steps require sudo. Please enter your password if prompted."
# add shell to /etc/shells if not present -
# allows it to be used as a login shell
if grep -Fq "$SHELL_PATH" /etc/shells; then
  echo "Local zsh already in /etc/shells!"
else
  echo "Adding zsh to /etc/shells..."
  echo "$SHELL_PATH" | sudo tee -a /etc/shells
fi

# only change the shell if it's not already zsh
if [ "$SHELL" = "$SHELL_PATH" ]; then
  echo "Shell is already zsh!"
else
  echo "Changing the default shell to zsh..."
  sudo usermod -s "$HOME/.nix-profile/bin/zsh" "$(whoami)"
  echo "Changed shell - please restart your shell after this script is complete for the change to take effect."
fi

SUDOERS_FILENAME="secure_path_includes_nix_$(whoami)"
SUDOERS_FULL_PATH="/etc/sudoers.d/$SUDOERS_FILENAME"
# as usual, only install if needed
if [ ! -f "$SUDOERS_FULL_PATH" ]; then
  echo "Installing modified sudoers secure_path..."
  # we can't just symlink to the original file generated by Home Manager,
  # since the systemd-networkd service is run with ProtectHome=true
  # which makes user homes are inaccessible, including from soft symlinks - hard symlinks are fine though,
  # but make replacing the file harder.
  sudo cp ./_internal/secure_path_includes_nix "$SUDOERS_FULL_PATH"
  sudo sed -i -e "s,SUBSTITUTE_USER,$(whoami),g;s,SUBSTITUTE_HOME,$HOME,g" "$SUDOERS_FULL_PATH"
  # sudoers files need to be owned by root, read-only, and have 0 "other" permissions
  sudo chown root:root "$SUDOERS_FULL_PATH"
  sudo chmod 440 "$SUDOERS_FULL_PATH"
  echo "sudoers file \"$SUDOERS_FULL_PATH\" installed."
else
  echo "sudoers file already installed!"
fi

# note: Updates to network rules will be handled from Home Manager,
# but adding new rules or removing old ones is impossible
# (due to requiring sudo, and there not being a mechanism to remove old rules)
NETWORK_RULE_SOURCE_DIR=~/".config/systemd/network"
for rule in "$NETWORK_RULE_SOURCE_DIR"/*; do
  RULE_BASENAME=$(basename "$rule")
  RULE_DESTINATION="/etc/systemd/network/$RULE_BASENAME"

  echo "Installing systemd network rule \"$RULE_BASENAME\"..."
  # copy the rule into place - requires sudo since the destination is owned by root
  sudo cp -f "$rule" "$RULE_DESTINATION"
  # once the file's in place, make it owned and writable by the current user so we can update it later
  sudo chown "$(whoami):$(id -gn)" "$RULE_DESTINATION"
  chmod 644 "$RULE_DESTINATION"
done

echo "Enabling systemd-networkd..."
sudo systemctl enable systemd-networkd
echo "Restarting systemd-networkd..."
sudo systemctl restart systemd-networkd
echo "Done!"
