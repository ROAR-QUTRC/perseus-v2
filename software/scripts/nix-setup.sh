#!/usr/bin/env bash

set -euo pipefail

# exit if run as root
if [ "$EUID" -eq 0 ]; then
  echo "Please run as yourself! Running as superuser (ie, with sudo) breaks the setup."
  exit
fi

echo "Setting up git submodule repos"

# cd to the repo root
cd "$(git rev-parse --show-toplevel)"

# clone submodules if not already done so workspace can actually build
git submodule update --init --recursive

if ! command -v nix &>/dev/null; then
  echo "Setting up Nix"
  # install nix! https://github.com/DeterminateSystems/nix-installer
  # using the Determinate Systems installer rather than the "official" installer for its uninstall options and better error handling
  curl --proto '=https' --tlsv1.2 -sSf -L https://install.determinate.systems/nix | sh -s -- install
else
  echo "Nix already present on system!"
fi

# Add the ROS overlay binary cache + our own cache to the trusted substituters and keys.
# Allows us to use them in the flake.
EXTRA_TRUSTED_SUBSTITUTERS="extra-trusted-substituters = https://roar-qutrc.cachix.org https://ros.cachix.org"
if grep -Fq "$EXTRA_TRUSTED_SUBSTITUTERS" /etc/nix/nix.conf; then
  echo "Extra trusted substituters already present!"
else
  echo "Adding extra trusted substituters..."
  echo "$EXTRA_TRUSTED_SUBSTITUTERS" | sudo tee -a /etc/nix/nix.conf
  RESTART_NIX_DAEMON=true
fi

EXTRA_TRUSTED_KEYS="extra-trusted-public-keys = roar-qutrc.cachix.org-1:ZKgHZSSHH2hOAN7+83gv1gkraXze5LSEzdocPAEBNnA= ros.cachix.org-1:dSyZxI8geDCJrwgvCOHDoAfOm5sV1wCPjBkKL+38Rvo="
if grep -Fq "$EXTRA_TRUSTED_KEYS" /etc/nix/nix.conf; then
  echo "Extra trusted keys already present!"
else
  echo "Adding extra trusted keys..."
  echo "$EXTRA_TRUSTED_KEYS" | sudo tee -a /etc/nix/nix.conf
  RESTART_NIX_DAEMON=true
fi

# restart Nix daemon so above changes take effect - but only if needed
if [ "${RESTART_NIX_DAEMON:-}" = true ]; then
  echo "Restarting nix daemon"
  sudo systemctl restart nix-daemon
fi

# add direnv hooks to bash and zsh if not already present
if grep -Fq 'direnv hook bash' ~/.bashrc &>/dev/null; then
  echo "bash direnv hook already set up!"
elif command -v bash &>/dev/null; then
  echo "Setting up bash direnv hook..."
  # shellcheck disable=SC2016
  echo 'eval "$(direnv hook bash)"' >>~/.bashrc
else
  echo "bash not detected on system, not setting up .bashrc"
fi

if grep -Fq 'direnv hook zsh' ~/.zshrc &>/dev/null; then
  echo "zsh direnv hook already set up!"
elif command -v zsh &>/dev/null; then
  echo "Setting up zsh direnv hook..."
  # shellcheck disable=SC2016
  echo 'eval "$(direnv hook zsh)"' >>~/.zshrc
else
  echo "zsh not detected on system, not setting up .zshrc"
fi

if grep -Fq 'hide_env_diff' ~/.config/direnv/direnv.toml &>/dev/null; then
  echo "direnv hide_env_diff already set up!"
else
  mkdir -p ~/.config/direnv
  touch ~/.config/direnv/direnv.toml
  echo "[global]" >>~/.config/direnv/direnv.toml
  echo "hide_env_diff = true" >>~/.config/direnv/direnv.toml
fi

# allow direnv to configure based on the .envrc file in the current directory
direnv allow

echo "Done!"
