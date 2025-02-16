#!/usr/bin/env bash
set -euo pipefail # Fail on errors, undefined vars, and pipe failures

# Install gstreamer
sudo apt-get install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libgstreamer-plugins-bad1.0-dev gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav gstreamer1.0-tools gstreamer1.0-x gstreamer1.0-alsa gstreamer1.0-gl gstreamer1.0-gtk3 gstreamer1.0-qt5 gstreamer1.0-pulseaudio

# Install rust to compile plugins Must restart terminal after installation
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh

# build the plugin
git clone https://gitlab.freedesktop.org/gstreamer/gst-plugins-rs.git
cd gst-plugins-rs/net/webrtc || exit
sudo apt install libssl-dev
cargo build
mkdir -p ~/.local/share/gstreamer-1.0/plugins/
cp ../../target/debug/libgstrswebrtc.so ~/.local/share/gstreamer-1.0/plugins/
