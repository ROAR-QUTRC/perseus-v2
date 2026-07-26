# Vision Pipeline and Detection Overlay (`detection_overlay`)

## Overview

The `perseus_vision` package runs its detectors and its image annotation as separate
concerns. Detectors read the camera stream and publish **detections only**; a single
`detection_overlay` node reads the camera stream plus every detector's detection topic
and produces **one** annotated image.

```text
              /camera/camera/color/image_raw
                │              │              │
       ┌────────┘              │              └──────────────┐
       ▼                       ▼                             ▼
 aruco_detector          cube_detector               detection_overlay
       │                       │                          ▲     ▲
       │                       │                          │     │
       └─ /perseus_vision/aruco/detections ───────────────┘     │
                               └─ /perseus_vision/cube/detections
                                                                │
                                        /perseus_vision/overlay/image
```

This replaces an earlier arrangement where each detector published its own annotated
image. That cost one full image encode and one image stream per detector, and no single
stream showed every detector's results.

:::{important}
The image path is never blocked waiting for detections.

Each incoming frame is annotated with the most recent detections **already received** and
republished immediately. The annotated stream therefore keeps the **source frame rate**,
while the boxes themselves lag by however long inference takes. Time-synchronising the
image against the detections would hold frames back to inference speed and defeat the
purpose, so it is deliberately not done.
:::

Detections older than `max_detection_age_s` stop being drawn, so a stalled or crashed
detector leaves a clean image rather than freezing its last boxes on screen.

---

## Composition

All three nodes are built as `rclcpp` components and are loaded into a single
`component_container_mt` by `perseus_vision.launch.py`, with
`use_intra_process_comms` enabled. Within one container the camera image is handed
between subscribers **by pointer** instead of being copied and serialised once per
subscriber.

```bash
ros2 launch perseus_vision perseus_vision.launch.py
```

Registered component plugins:

```text
perseus_vision::ArucoDetector
perseus_vision::CubeDetector
perseus_vision::DetectionOverlay
```

Each also has a standalone executable for running one node on its own:

```bash
ros2 run perseus_vision aruco_detector_node
ros2 run perseus_vision cube_detector
ros2 run perseus_vision detection_overlay_node
```

:::{note}
Intra-process comms only applies **within** a container. Running the nodes as separate
executables is still supported and behaves identically, but each process decodes the
image independently.
:::

---

## The detection message

Detectors publish `perseus_interfaces/msg/DetectionArray`, one message per processed
frame. `header.stamp` and `header.frame_id` are copied from the source image, so the
bounding polygons are in that image's pixel coordinates **and** any 3D poses are in that
camera's optical frame. One header, one frame, both halves consistent.

Each `Detection` carries:

| Field          | Type                          | Description                                                                 |
| -------------- | ----------------------------- | --------------------------------------------------------------------------- |
| `bounding_box` | `geometry_msgs/Polygon`       | Corner points in image pixels. Implicitly closed, so 4 points draw 4 edges. |
| `color`        | `std_msgs/ColorRGBA`          | Render colour, components in `[0.0, 1.0]`. Chosen by the detector.          |
| `class_label`  | `string`                      | Human-readable label, e.g. `aruco_12`, `cube_blue`.                         |
| `id`           | `int32`                       | ArUco marker ID, or cube class index. Saves consumers parsing the label.    |
| `confidence`   | `float32`                     | Model certainty in `[0.0, 1.0]`.                                            |
| `has_pose`     | `bool`                        | Whether `pose` holds a usable 3D result.                                    |
| `pose`         | `geometry_msgs/Pose`          | 3D pose in the header's frame. Only meaningful when `has_pose` is true.     |
| `source_image` | `sensor_msgs/CompressedImage` | Optional detection crop for offline debugging. Normally left empty.         |

:::{important}
**Poses are published in the camera optical frame, not in `odom`.**

Consumers transform into `odom` or `map` themselves via tf2. This is deliberate: it keeps
one `frame_id` correct for both the polygon and the pose, it lets the consumer choose the
transform lookup time rather than inheriting the detector's, and it means a missing or
stale TF chain no longer silently empties the detection topic.

This replaces an earlier split where a separate `ObjectDetections` message carried
`odom`-frame poses on its own topic.
:::

**`has_pose` rather than dropping the detection.** A cube whose depth read fails, an ArUco
marker seen without calibration, or any detector in a 2D-only mode still reports the
detection with `has_pose: false`. "I can see it but cannot place it" is actionable for a
consumer; silently omitting it is not.

**Why a polygon rather than `vision_msgs/Detection2DArray`.** The framework standards say
to prefer a pre-existing ROS message where one fits. `vision_msgs/BoundingBox2D` describes
a centre point, a size and a rotation, which is a rotated rectangle. An ArUco marker seen
at an angle projects to a general quadrilateral, which a rotated rectangle cannot
represent, so a polygon is used instead.

:::{warning}
`source_image` embeds a full compressed image inside every detection it is set on. Leave
it empty in normal operation, otherwise the size of each detection message becomes
unbounded and unpredictable.
:::

**The colour is chosen by the detector, not the overlay.** This keeps the overlay
independent of how many detector types exist. If a detection arrives with alpha `0.0`,
the overlay treats the colour as unset and falls back to green so the box is still
visible.

### The detection service

`DetectObjects` returns the same shape — a `Header` plus `Detection[]`, plus a
human-readable `message`. A service caller and a topic subscriber therefore see identical
data.

---

## Node name

```text
detection_overlay
```

## Subscribed topics

### Image input

Raw, used when `compressed_io = false`:

```text
<input_image_topic>   (sensor_msgs/Image)
```

Compressed, used when `compressed_io = true`:

```text
<input_image_topic>/compressed   (sensor_msgs/CompressedImage)
```

### Detections

One subscription per entry in `detection_topics`:

```text
<detection_topics[i]>   (perseus_interfaces/msg/DetectionArray)
```

## Published topics

```text
<output_image_topic>              (sensor_msgs/Image)
<output_image_topic>/compressed   (sensor_msgs/CompressedImage)   # when compressed_io = true
```

---

## Parameters and defaults

All parameters are under:

```yaml
detection_overlay:
  ros__parameters: ...
```

| Parameter             |     Type |                                                                       Default | Description                                                                                   |
| --------------------- | -------: | ----------------------------------------------------------------------------: | --------------------------------------------------------------------------------------------- |
| `input_image_topic`   |   string |                                              `/camera/camera/color/image_raw` | Camera image to annotate.                                                                     |
| `output_image_topic`  |   string |                                              `/perseus_vision/overlay/image`  | Annotated image output.                                                                       |
| `detection_topics`    | string[] | `[/perseus_vision/aruco/detections, /perseus_vision/cube/detections]`          | Detection topics to overlay. Add an entry to include a new detector.                          |
| `max_detection_age_s` |   double |                                                                         `1.0` | Detections older than this are not drawn, so a stalled detector does not freeze boxes on screen. |
| `compressed_io`       |     bool |                                                                       `false` | Subscribe and publish `<topic>/compressed` as `CompressedImage`.                              |
| `show_staleness`      |     bool |                                                                       `false` | Draw the detection count and the age of the oldest detection onto the image.                  |

---

## Adding a new detector

1. Publish a `DetectionArray` from the new node, with `header` copied from the source
   image and a `color`, `class_label`, `id` and `has_pose` set per detection.
2. Add the new topic to `detection_topics` in `config/perseus_vision.yaml`.
3. If the node should share the container, register it as a component and add it to
   `perseus_vision.launch.py`.

No change to `detection_overlay` itself is required.

---

## Rendering

Both the overlay node and the detector capture services draw through the same
implementation, `perseus_vision/common/detection_renderer.hpp`, so a captured PNG looks
identical to the live stream.

Detectors do **not** annotate on the live path. They cache the raw frame alongside their
own detections and render only when a capture is actually requested, so annotation costs
nothing while nobody is capturing.

:::{note}
The ArUco detector no longer draws 3D pose axes. Axes cannot be reconstructed from a
polygon, since the overlay has neither the camera intrinsics nor the rotation and
translation vectors. The `axis_length` parameter was removed with them.
:::

---

## Troubleshooting

**The overlay image publishes but has no boxes.** Check that detections are actually
arriving and are recent:

```bash
ros2 topic hz /perseus_vision/aruco/detections
ros2 topic echo --once /perseus_vision/aruco/detections
```

If detections are arriving but nothing is drawn, their age most likely exceeds
`max_detection_age_s`. Set `show_staleness: true` to see the measured age on the image.

**The overlay rate is lower than the camera rate.** The image subscription uses a queue
depth of 1, so frames are dropped rather than queued when the node cannot keep up. A
persistently low rate means annotation or JPEG re-encoding is the bottleneck.

**Detections appear at the wrong scale or position.** The overlay draws polygons in the
coordinates the detector published. Confirm the detector and the overlay are consuming
the same camera topic and resolution.
