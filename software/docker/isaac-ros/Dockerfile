# perseus-lite Isaac ROS overlay image (ROS 2 Humble, Jetson Orin Nano / x86).
#
# FROM an official Isaac ROS 3.2 NGC base (which already carries CUDA, TensorRT,
# cuDNN, the ROS 2 Humble desktop, and the NVIDIA apt sources), then layer the
# specific Isaac ROS perception packages this robot uses plus a small relay
# package that converts Isaac-specific messages to standard types for bridging.
#
# Build is Jetson/GPU-only and needs `docker login nvcr.io`; it is NOT run in CI
# (see .github/workflows/all.yaml — only `docker compose config` is validated).

ARG ISAAC_ROS_IMAGE
FROM ${ISAAC_ROS_IMAGE}

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

# Isaac ROS perception packages + a mono camera driver. Package names follow the
# `ros-humble-isaac-ros-<pkg>` convention from the NVIDIA Isaac apt repo; confirm
# against the 3.2 apt index for your JetPack if a name has drifted.
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-isaac-ros-apriltag \
    ros-humble-isaac-ros-yolov8 \
    ros-humble-isaac-ros-tensor-rt \
    ros-humble-isaac-ros-dnn-image-encoder \
    ros-humble-isaac-ros-image-proc \
    ros-humble-usb-cam \
    ros-humble-image-transport-plugins \
    ros-humble-topic-tools \
    && rm -rf /var/lib/apt/lists/*

# YOLOv8 model, reused verbatim from perseus_vision (same URL + SHA256 as
# software/ros_ws/src/perseus_vision/CMakeLists.txt). The GPU-specific TensorRT
# engine is NOT built here — it is arch-specific and produced on first run by
# entrypoint.sh into a named volume.
ARG MODEL_URL=https://github.com/ROAR-QUTRC/perseus-v2/releases/download/models-v1/cube_detector_yolob8s.onnx
ARG MODEL_SHA256=1245a1285628839cac4a86dc1380c55b64bc8f764b4190c1c6e7162dafca55f7
RUN mkdir -p /opt/perseus/models \
    && curl -fSL "${MODEL_URL}" -o /opt/perseus/models/cube_detector_yolob8s.onnx \
    && echo "${MODEL_SHA256}  /opt/perseus/models/cube_detector_yolob8s.onnx" | sha256sum -c -

# Build the in-image relay/launch overlay (Humble-only; never seen by the host
# colcon workspace — it lives outside software/ros_ws on purpose).
COPY ws /opt/perseus/ws
RUN source /opt/ros/humble/setup.bash \
    && cd /opt/perseus/ws \
    && colcon build --symlink-install

COPY entrypoint.sh /opt/perseus/entrypoint.sh
RUN chmod +x /opt/perseus/entrypoint.sh

ENTRYPOINT ["/opt/perseus/entrypoint.sh"]
CMD ["ros2", "launch", "perseus_isaac_relay", "perseus_isaac.launch.py"]
