# ArUco Detector Node (`aruco_detector`)

## Overview

`aruco_detector` detects **OpenCV ArUco markers** in a camera stream and estimates each marker's **6-DoF pose** using known camera intrinsics + marker size. It can:

- Subscribe to **raw** or **compressed** images
- Optionally subscribe to **CameraInfo** for live calibration
- Filter weak detections using a **minimum bounding-box area** threshold
- Publish **detections** (`DetectionArray`) carrying both the image-space polygon and,
  when calibration allows, the 3D pose
- Publish rviz visualization markers
- Broadcast TF frames for each detected marker (`aruco_marker_<id>`) (optional)
- Provide detections via a **service**, with an option to **save an annotated image to disk**

:::{important}
This node does **not** publish an annotated image stream. Annotation is handled by
`detection_overlay`, which combines this node's detections with every other detector's
into a single image. See [the vision pipeline page](project:/development/software/vision_pipeline.md).
:::

---

## Node Name

```text
aruco_detector
```

---

## Subscribed Topics

### Image input (raw)

Used when `compressed_io = false`:

```text
<input_img>   (sensor_msgs/Image)
```

**Default**

```text
/camera/camera/color/image_raw
```

---

### Image input (compressed)

Used when `compressed_io = true`:

```text
<input_img>/compressed   (sensor_msgs/CompressedImage)
```

---

### Camera info (optional)

Used when `use_camera_info = true`, in both the raw and compressed image modes:

```text
<camera_info_topic>   (sensor_msgs/CameraInfo)
```

**Default**

```text
/camera/camera/color/camera_info
```

---

## Published Topics

### Detections

Always published, on a single topic. Consumed by `detection_overlay` for drawing, and by
anything wanting marker poses:

```text
<output_topic>   (perseus_interfaces/msg/DetectionArray)
```

**Default**

```text
/perseus_vision/aruco/detections
```

Each detection carries the marker's **perspective quad** as a 4-point polygon, a
`class_label` of `aruco_<id>`, the numeric marker `id`, a `confidence` of `1.0`, and the
marker `pose` in the camera optical frame with `has_pose: true`.

ArUco detection is geometric rather than probabilistic: a marker either decodes to a valid
ID or it is not reported at all, so an accepted marker is always full confidence.

:::{important}
The pose is expressed in `camera_frame`, not in `odom`. Transform it yourself via tf2, or
use the `aruco_marker_<id>` TF frames below. See
[the vision pipeline page](project:/development/software/vision_pipeline.md) for why.
:::

If no camera calibration is available the marker is still reported, with `has_pose: false`
and no TF broadcast.

---

### Visualization markers

Always published:

```text
/detection/aruco/markers   (visualization_msgs/MarkerArray)
```

---

## TF Frames

If `publish_tf = true`, for every marker whose pose was resolved, the node broadcasts:

```text
camera_frame  ->  aruco_marker_<N>
```

The transform that was actually measured is published as-is, parented to the camera
frame, and tf2 composes it with the rest of the tree. A consumer wanting the marker in
`odom` or `map` looks that up itself, which also lets it choose the lookup time.

---

## Services

### Detect Objects

```text
/detect_objects   (perseus_interfaces/srv/DetectObjects)
```

Returns the **latest cached detections** in exactly the shape published on the topic:

- `header`: stamp and frame of the processed image
- `detections[]`: `Detection` entries, each with polygon, label, id, confidence, and
  `pose` when `has_pose` is true
- `message`: human-readable status

#### Image capture feature

The request supports an image capture mode:

- `request->capture_image` (bool)
- `request->img_save_path` (string path)

If `capture_image = true`, the node will:

1. Create the directory `img_save_path` if it does not exist
2. Annotate a copy of the latest **raw** frame with the cached detections, using the same
   renderer the overlay node uses, so the capture matches the live stream
3. Overlay text on the image:
   - Human-readable timestamp (system clock)
   - Marker coordinate summary (`X, Y, Z` derived from the translation vector)

4. Save a PNG named `aruco_<id1>_<id2>..._<epoch_ms>.png`, or
   `aruco_no_markers_<epoch_ms>.png` when nothing was detected

:::{note}
The frame is annotated **on demand**, not kept permanently annotated. Nothing is drawn on
the live detection path while no capture is in progress.
:::

---

## Parameters and Defaults

All parameters are under:

```yaml
aruco_detector:
  ros__parameters: ...
```

### Marker detection / pose estimation

| Parameter               |   Type | Default | Description                                                                                          |
| ----------------------- | -----: | ------: | ---------------------------------------------------------------------------------------------------- |
| `marker_length`         | double |  `0.35` | Physical marker size (meters). Used for pose estimation scale.                                       |
| `dictionary_id`         |    int |     `1` | OpenCV predefined dictionary enum value (must match the printed markers).                            |
| `min_bounding_box_area` | double | `100.0` | Filters detections: marker's 2D bounding box area in pixels must be ≥ this threshold to be accepted. |

**Bounding box filtering details**

- For each detected marker's 4 corner points, the node computes:
  - `min_x, max_x, min_y, max_y`
  - area = `(max_x - min_x) * (max_y - min_y)`

- If area < `min_bounding_box_area`, the marker is ignored entirely (no detection, no pose, no TF).

This helps reject:

- tiny far-away false positives
- noisy corner detections
- partially detected markers

---

### Frames / transforms

| Parameter         |   Type |                       Default | Description                                                                                |
| ----------------- | -----: | ----------------------------: | ------------------------------------------------------------------------------------------ |
| `camera_frame`    | string | `camera_color_optical_frame`  | Frame the marker poses are measured in, and the TF parent of `aruco_marker_<id>`. Should be the camera optical frame. |

---

### Image input

| Parameter       |   Type |                          Default | Description                                                    |
| --------------- | -----: | -------------------------------: | ---------------------------------------------------------------- |
| `input_img`     | string | `/camera/camera/color/image_raw` | Raw image topic (base).                                        |
| `compressed_io` |   bool |                          `false` | If true, subscribe to `<input_img>/compressed` instead.        |

---

### Outputs

| Parameter                     |   Type |                              Default | Description                                                       |
| ----------------------------- | -----: | -----------------------------------: | ----------------------------------------------------------------- |
| `publish_tf`   |   bool |                             `true` | Broadcast `camera_frame -> aruco_marker_<id>` transforms. |
| `output_topic` | string | `/perseus_vision/aruco/detections` | Detection output topic name.                              |

---

### Camera calibration

Two modes are supported.

#### 1) Parameter-based calibration (always initialized)

These parameters are always declared and used as initial calibration.

| Parameter                 |      Type |                                   Default | Description                                      |
| ------------------------- | --------: | ----------------------------------------: | ------------------------------------------------ |
| `camera_matrix`           | double[9] | `[530.4, 0, 320, 0, 530.4, 240, 0, 0, 1]` | Row-major intrinsic matrix K.                    |
| `distortion_coefficients` |  double[] |                             `[0,0,0,0,0]` | Distortion coeffs, typically `[k1,k2,p1,p2,k3]`. |

If `camera_matrix` does not contain exactly 9 elements the node logs an error and falls
back to the default matrix.

#### 2) CameraInfo override (optional)

If enabled, incoming `CameraInfo` replaces intrinsics.

| Parameter           |   Type |                            Default | Description                                                                            |
| ------------------- | -----: | ---------------------------------: | -------------------------------------------------------------------------------------- |
| `use_camera_info`   |   bool |                            `false` | If true, subscribe to `camera_info_topic` and overwrite camera intrinsics dynamically. |
| `camera_info_topic` | string | `/camera/camera/color/camera_info` | Topic for `sensor_msgs/CameraInfo`.                                                    |

**CameraInfo behavior**

- The intrinsic matrix is rebuilt from `msg->k[0..8]`
- The distortion coefficients are rebuilt from `msg->d[]` (any length supported)

> If the intrinsic matrix is ever empty, pose estimation is skipped and a warning is
> logged once. Because it is initialized from parameters, this will not happen unless it
> is cleared elsewhere.

---

## Detection Pipeline

1. Receive image (raw or compressed)
2. Detect markers with:

   ```cpp
   _detector.detectMarkers(frame, corners, ids);
   ```

3. Clear cached detections and record the new timestamp
4. If markers exist, for each marker:
   - estimate pose via `cv::solvePnP` using 3D marker corner points and detected 2D image points
   - compute bounding box area in pixels and apply the `min_bounding_box_area` filter
   - append a `Detection` holding the marker's perspective quad, colour, label and id
   - if calibration is available, solve the pose, set `has_pose`, and broadcast TF

5. Cache the **raw** frame and the detections, for the capture service
6. Publish the visualization markers and the `DetectionArray`

---

## Example YAML Configuration

```yaml
aruco_detector:
  ros__parameters:
    marker_length: 0.35
    # 4x4: 50=0, 100=1, 250=2, 1000=3 | 5x5: 50=4, 100=5, 250=6, 1000=7 | 6x6: 50=8, 100=9, 250=10, 1000=11
    dictionary_id: 1
    min_bounding_box_area: 150.0

    camera_frame: camera_color_optical_frame

    input_img: /camera/camera/color/image_raw
    compressed_io: false

    publish_tf: true
    output_topic: /perseus_vision/aruco/detections

    use_camera_info: true
    camera_info_topic: /camera/camera/color/camera_info
```

---

## Usage

### Run the whole pipeline

Preferred, since it also starts the overlay:

```bash
ros2 launch perseus_vision perseus_vision.launch.py
```

### Run this node on its own

```bash
ros2 run perseus_vision aruco_detector_node --ros-args \
  --params-file <path_to_yaml>
```

### Service call (detections only)

```bash
ros2 service call /detect_objects perseus_interfaces/srv/DetectObjects "{}"
```

### Service call (capture image)

```bash
ros2 service call /detect_objects perseus_interfaces/srv/DetectObjects \
"{capture_image: true, img_save_path: '/tmp/aruco_captures'}"
```

---

## Notes

- `dictionary_id` must match the marker dictionary used to generate/print the tags.
- `marker_length` must match the real marker size in meters.
- Filtering by `min_bounding_box_area` is in **pixels²**, so thresholds depend on:
  - camera resolution
  - distance to marker
  - FOV and lens

- To see the detections drawn on the camera stream, subscribe to
  `/perseus_vision/overlay/image`.
