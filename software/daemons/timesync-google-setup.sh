#!/bin/bash
set -e  # Exit immediately on any error

SERVICE_NAME="timesync-google.service"
SERVICE_SRC="$(cd "$(dirname "$0")" && pwd)/$SERVICE_NAME"
SERVICE_DEST="/etc/systemd/system/$SERVICE_NAME"

# Check the service file exists next to this script
if [ ! -f "$SERVICE_SRC" ]; then
    echo "ERROR: $SERVICE_NAME not found next to this script ($SERVICE_SRC)"
    exit 1
fi

echo "Installing service..."
sudo cp "$SERVICE_SRC" "$SERVICE_DEST"
sudo chmod 644 "$SERVICE_DEST"

echo "Reloading daemons..."
sudo systemctl daemon-reload

echo "Enabling service for persistence..."
sudo systemctl enable "$SERVICE_NAME"

echo "All done. Rebooting in 3 seconds... (Ctrl+C to cancel)"
sleep 3
sudo reboot
