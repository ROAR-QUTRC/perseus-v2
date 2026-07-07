#!/usr/bin/env bash
# Container entrypoint: source the ROS overlay, ensure a TensorRT engine exists
# (built on first run and cached in the models volume, since engines are
# GPU-arch-specific and cannot be baked into the image), then exec the command.
set -euo pipefail

# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source /opt/perseus/ws/install/setup.bash

model_dir="/opt/perseus/models"
onnx="${model_dir}/cube_detector_yolob8s.onnx"
engine="${model_dir}/cube_detector_yolob8s.plan"

if [[ -f "${onnx}" && ! -f "${engine}" ]]; then
    echo "[entrypoint] Building TensorRT engine (first run; slow on Orin Nano)..."
    if command -v trtexec >/dev/null 2>&1; then
        trtexec="trtexec"
    elif [[ -x /usr/src/tensorrt/bin/trtexec ]]; then
        trtexec="/usr/src/tensorrt/bin/trtexec"
    else
        trtexec=""
        echo "[entrypoint] WARNING: trtexec not found; skipping engine build." >&2
    fi
    if [[ -n "${trtexec}" ]]; then
        "${trtexec}" --onnx="${onnx}" --saveEngine="${engine}" --fp16 \
            || echo "[entrypoint] WARNING: engine build failed; DNN pipeline disabled." >&2
    fi
fi

exec "$@"
