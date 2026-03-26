#!/usr/bin/env bash

if [ "$EUID" -eq 0 ]; then
  echo "Error: Please run this script as a non-root user"
  exit 1
fi

sudo rm -f /dev/v4l/by-id/fake-symlink-*

all_devices="$(ls /dev/video*)"
echo "$all_devices"

symlinked="$(find /dev/v4l/by-id -type l -printf '%l\n')"
echo "$symlinked"

# Build array of available (non-symlinked) devices
available=()
while IFS= read -r device; do
    # Extract the videoX name from the full path
    name=$(basename "$device")
    # Check if this name appears anywhere in the symlinked string
    if ! grep -q "$name" <<< "$symlinked"; then
        available+=("$name")
    fi
done <<< "$all_devices"

camera_index=0

echo "Found missing links for:"
echo "${available[@]}"

for device in "${available[@]}"; do
    sudo ln -s "/dev/$device" "/dev/v4l/by-id/fake-symlink-$(v4l2-ctl -d /dev/$device --info | grep "Card type" | awk -F': ' '{print $2}' | tr ' ' '_')_$((camera_index / 2))-video-index$(($camera_index % 2))"
    echo "Linking /dev/$device to /dev/v4l/by-id/fake-symlink-$(v4l2-ctl -d /dev/$device --info | grep "Card type" | awk -F': ' '{print $2}' | tr ' ' '_')_$((camera_index / 2))-video-index$(($camera_index % 2))"
    ((camera_index++))
done