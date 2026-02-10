#!/usr/bin/env bash

set -euo pipefail

# exit if run as root
if [ "$EUID" -eq 0 ]; then
  echo "Please run as yourself! Running as superuser (ie, with sudo) breaks the setup."
  exit 1
fi

FIRMWARE_CONFIG=/boot/firmware/config.txt
PCIE_RULE="dtparam=pciex1"
PCIE_GEN_RULE="dtparam=pciex1_gen=3"
if grep -Fq "$PCIE_RULE" "$FIRMWARE_CONFIG" &>/dev/null; then
  echo "Setting up PCIE"
  echo "$PCIE_RULE" | sudo tee -a "$FIRMWARE_CONFIG"
else
  echo "PCIE already set up - continuing"
fi
if grep -Fq "$PCIE_GEN_RULE" "$FIRMWARE_CONFIG" &>/dev/null; then
  echo "Setting up Gen 3 PCIE"
  echo "$PCIE_GEN_RULE" | sudo tee -a "$FIRMWARE_CONFIG"
else
  echo "PCIE Gen rule already set up - continuing"
fi

echo "Updating and upgrading all packages and kernel"
sudo apt-get update
sudo apt-get upgrade
sudo apt-get full-upgrade

echo "Installing iwlwifi firmware"
sudo apt-get firmware-iwlwifi
