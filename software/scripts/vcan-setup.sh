#!/usr/bin/env bash

CAN_IF="${1:-vcan0}"

# Set up virtual CAN interface vcan0
sudo modprobe can
sudo modprobe vcan
sudo modprobe can-raw
sudo ip link add dev "${CAN_IF}" type vcan
sudo ip link set up "${CAN_IF}"
