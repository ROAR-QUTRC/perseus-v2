#!/usr/bin/env bash
set -euo pipefail

# Set up virtual CAN interface vcan0
sudo modprobe can
sudo modprobe vcan
sudo modprobe can-raw
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
