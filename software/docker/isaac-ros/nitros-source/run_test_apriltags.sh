#!/usr/bin/env bash
# `pixi run -e isaac-nitros test-apriltags` — live-camera AprilTag test.
# Launches our own apriltag_camera_launch.py (usb_cam -> rectify ->
# apriltag) rather than NVIDIA's isaac_ros_apriltag_usb_cam.launch.py, which
# has a resolution/calibration mismatch (see camera_c920_params.yaml). See
# README.md "Live camera test" for the RViz-on-a-different-machine setup.
set -eo pipefail
# NOTE: no `set -u` -- colcon's generated install/setup.bash isn't
# nounset-safe (references e.g. $COLCON_TRACE without a default).
cd "$(dirname "${BASH_SOURCE[0]}")"

if [ ! -f ws/install/setup.bash ]; then
  echo "ws/install/setup.bash not found -- build the Stage 4 workspace first" \
       "(see README.md Stage 0/1/3/4 commands)." >&2
  exit 1
fi

GXF_LIB_DIRS=$(find ws/install -type d -path "*/gxf/lib/*" ! -name test | tr '\n' ':')
INSTALL_LIB_DIRS=$(find ws/install -maxdepth 2 -type d -name lib | tr '\n' ':')
EXTRA_LIB_DIRS="/opt/nvidia/vpi4/lib/aarch64-linux-gnu:/opt/nvidia/cvcuda0/lib:"

# shellcheck disable=SC1091
source ws/install/setup.bash
export LD_LIBRARY_PATH="${GXF_LIB_DIRS}${INSTALL_LIB_DIRS}${EXTRA_LIB_DIRS}${LD_LIBRARY_PATH:-}"

echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-<unset>} RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-<unset>}"
echo "Your laptop's RViz must use the SAME domain ID and a compatible RMW" \
     "(cyclonedds recommended) to see these topics -- see README.md."

exec ros2 launch apriltag_camera_launch.py
