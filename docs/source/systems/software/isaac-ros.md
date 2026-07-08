# Isaac ROS (CUDA / NITROS) on the Jetson

GPU-accelerated perception — AprilTag detection now, TensorRT object detection
next — runs on the Jetson Orin Nano via NVIDIA **Isaac ROS**, in Docker, and
feeds results to the native robot stack.

Scaffold lives in `software/docker/isaac-ros/`. It is validated by
`docker compose config`, lint, and the TUI tests, but has **not yet been
GPU-validated on hardware** — the build/run steps below are the on-Jetson
bring-up runbook.

This robot's Jetson has since moved to **JetPack 7**, which NVIDIA's Isaac
ROS 4.x line doesn't yet support on Orin (Thor-only today). The runbook below
still targets JetPack 6.x per NVIDIA's shipped binaries. A separate,
experimental from-source build attempt for Orin+JetPack7 is tracked in
[isaac-ros-nitros-source-build.md](isaac-ros-nitros-source-build.md) — not
required reading unless you're picking up that effort.

## Why containers, and why a bridge

Two hard facts shape the whole design:

1. The **Orin Nano cannot run current Isaac ROS 4.x** (that line is Jetson
   Thor–only, JetPack 7). Its terminal release is **Isaac ROS 3.2 = ROS 2
   Humble on JetPack 6.x**. There is no Jazzy build for this board.
2. Our host stack is **Jazzy**, and **Humble ↔ Jazzy do not interoperate over
   raw DDS** — since Iron, ROS 2 changed serialization and added type
   descriptions/hashes, so a matching RMW and `ROS_DOMAIN_ID` get you discovery
   but not correct data (messages are silently dropped).

So Isaac ROS runs in Humble containers on an **isolated ROS domain (52)**, and
two [`zenoh-bridge-ros2dds`](https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds)
services tunnel a small allow-list of standard-typed topics to the host on
domain **51**. The two DDS graphs never cross-discover; only zenoh crosses,
over loopback TCP.

```{graphviz}
:caption: Isaac ROS (Humble, domain 52) bridged to the Jazzy host (domain 51)
:align: center

digraph isaac_bridge {
    graph [rankdir=LR, bgcolor="transparent", fontname="Roboto", nodesep=0.35, ranksep=0.7];
    node  [fontname="Roboto", fontsize=11, style="filled,rounded", shape=box, penwidth=1.1, fontcolor="white"];
    edge  [fontname="Roboto", fontsize=9, color="#7a6cad"];

    subgraph cluster_ctr {
        label="Docker (ROS 2 Humble, domain 52)"; fontname="Roboto"; color="#37474f";
        isaac [label="isaac\ncamera → rectify → apriltag\n(+ yolov8 phase 2) + relay", fillcolor="#1a237e"];
        bh    [label="bridge-humble\nzenoh (listen)", fillcolor="#00695c"];
    }
    subgraph cluster_host {
        label="Native host (ROS 2 Jazzy, domain 51)"; fontname="Roboto"; color="#37474f";
        bj   [label="bridge-jazzy\nzenoh (connect)", fillcolor="#00695c"];
        host [label="perseus-lite stack\n(Nav2, TUI, …)", fillcolor="#ec407a"];
    }

    isaac -> bh [label="DDS @52"];
    bh -> bj [label="zenoh tcp/127.0.0.1:7447", dir=both, penwidth=2.0, color="#ec407a"];
    bj -> host [label="DDS @51"];
}
```

## Topic contract

Only these `/perseus_isaac/*` topics cross the bridge (one-way, container →
host, in v1). No Isaac-specific message type is ever bridged — the relay node
(`perseus_isaac_relay`) converts them to standard types first.

| Topic                                   | Type                              | Notes                           |
| --------------------------------------- | --------------------------------- | ------------------------------- |
| `/perseus_isaac/apriltag/poses`         | `geometry_msgs/PoseArray`         | tag poses, camera frame         |
| `/perseus_isaac/apriltag/detections`    | `vision_msgs/Detection2DArray`    | tag id in `hypothesis.class_id` |
| `/perseus_isaac/debug/image/compressed` | `sensor_msgs/CompressedImage`     | throttled ≤ 2 Hz                |
| `/perseus_isaac/health`                 | `diagnostic_msgs/DiagnosticArray` | relay heartbeat, 1 Hz           |
| `/perseus_isaac/cube/detections`        | `vision_msgs/Detection2DArray`    | **phase 2** — object detection  |

Never bridged: raw/rectified images, `camera_info`, `/tf`. `vision_msgs` has the
same nested layout in Humble and Jazzy 4.x, but confirm on first bring-up.

## Files

```
software/docker/isaac-ros/
├── .env                     # image pins, domains (52/51), camera device
├── compose.yaml             # isaac + bridge-humble + bridge-jazzy
├── Dockerfile               # FROM NGC Isaac ROS Humble base + perception pkgs
├── entrypoint.sh            # source overlay; first-run TensorRT engine cache
├── config/
│   ├── zenoh-bridge-humble.json5   # domain 52, listen, /perseus_isaac/* allow
│   ├── zenoh-bridge-jazzy.json5    # domain 51, connect, same allow-list
│   └── camera_info/                # camera calibration (placeholder + README)
└── ws/src/perseus_isaac_relay/     # Humble-only pkg, built ONLY in the image
```

`.env` is the single source of truth for image tags, `ISAAC_DOMAIN_ID=52`,
`HOST_DOMAIN_ID=51`, `ZENOH_PORT`, and `CAMERA_DEVICE`.

## Jetson bring-up runbook

Host prep (once):

1. **JetPack 6.x** — `cat /etc/nv_tegra_release`.
2. **nvidia container runtime as default** — set `"default-runtime": "nvidia"`
   in `/etc/docker/daemon.json`, then `sudo systemctl restart docker`.
3. **Power + memory** — `sudo nvpmodel -m 0` (MAXN/Super) and add zram/swap; the
   8 GB is shared CPU/GPU (see budget below).
4. **NGC login** — `docker login nvcr.io` with an NGC API key (the Isaac base is
   auth-gated; this is why CI never builds the image).

Deploy:

```console
cd software/docker/isaac-ros
docker compose build          # slow first time
docker compose up -d
docker compose logs -f
```

On the **first run**, `entrypoint.sh` builds the arch-specific TensorRT engine
into the `isaac-models` volume — this takes several minutes and is memory-hungry;
it is cached for subsequent starts.

Verify from the host (native Jazzy/Pixi stack):

```console
ros2 topic hz /perseus_isaac/apriltag/detections
ros2 topic echo /perseus_isaac/health
```

The TUI wraps the compose commands as tasks: `isaac_build`, `isaac_up`,
`isaac_down`, `isaac_logs`, `isaac_status` (Tasks tab).

## Orin Nano 8 GB budget

8 GB is shared between CPU and GPU. Keep to **AprilTag + at most one** TensorRT
pipeline at a time, run headless, and enable zram. Publishing only compact
results across the bridge (not raw imagery) is deliberate — it keeps the heavy
NITROS work intra-process on the GPU and off the wire.

## CSI camera variant

The default graph uses `usb_cam` on `${CAMERA_DEVICE}`. For a CSI camera, swap
it for the Argus path (`isaac_ros_argus_camera` / `nvarguscamerasrc`) in
`perseus_isaac.launch.py`, drop the `devices:` passthrough in `compose.yaml`,
and expose `/dev/nvhost-*` / the Argus socket instead. The rest of the graph is
unchanged.

## Relationship to `perseus_vision`

`perseus_vision` (the OpenCV/ONNX cube + ArUco detector) stays the **x86** path —
it builds only in the `machine-learning` Pixi env and is not built on aarch64.
This Isaac ROS container is the **Jetson/GPU** answer to the same problem, and
the phase-2 object detector reuses the very same YOLOv8 cube model (same URL and
SHA256). AprilTag here is the GPU successor to the ArUco detector.
