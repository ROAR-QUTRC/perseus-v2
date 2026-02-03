# ArUco Detector Node (`aruco_detector`)

## Overview

`aruco_detector` detects **OpenCV ArUco markers** in a camera stream and estimates each marker’s **6-DoF pose** using known camera intrinsics + marker size. It can:

- Subscribe to **raw** or **compressed** images
- Optionally subscribe to **CameraInfo** for live calibration
- Filter weak detections using a **minimum bounding-box area** threshold
- Publish **annotated debug images**
- Publish detections on a topic (`ObjectDetections`) (optional)
- Broadcast TF frames for each detected marker (`aruco_marker_<id>`) (optional)
- Provide detections via a **service**, with an option to **save an annotated image to disk**

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

Used only when `use_camera_info = true` **and** `compressed_io = false` (as implemented):

```text
<camera_info_topic>   (sensor_msgs/CameraInfo)
```

**Default**

```text
/camera/camera/color/camera_info
```

> Note: In the current code, `camera_info` subscription is created only in the raw-image branch (not in the compressed branch).

---

## Published Topics

### Annotated output image (raw)

Published when `publish_img = true` and `compressed_io = false`:

```text
<output_img>   (sensor_msgs/Image)
```

**Default**

```text
/detection/aruco/image
```

---

### Annotated output image (compressed)

Published when `publish_img = true` and `compressed_io = true`:

```text
<output_img>/compressed   (sensor_msgs/CompressedImage)
```

---

### Detection output topic (optional)

Published when `publish_output = true`:

```text
<output_topic>   (perseus_interfaces/msg/ObjectDetections)
```

**Default**

```text
/detection/aruco/detections
```

---

## TF Frames

If `publish_tf = true`, for every detected marker ID `N`, the node publishes:

```text
aruco_marker_<N>
```

Parent frame:

```text
tf_output_frame
```

**Default parent frame**

```text
odom
```

---

## Services

### Detect Objects

```text
/detect_objects   (perseus_interfaces/srv/DetectObjects)
```

Returns the **latest cached detections**, including:

- `ids[]`: marker IDs
- `poses[]`: marker poses in `tf_output_frame`
- `stamp`: timestamp of the processed frame
- `frame_id`: set to `tf_output_frame`

#### Image capture feature (new)

The request supports an image capture mode:

- `request->capture_image` (bool)
- `request->img_save_path` (string path)

If `capture_image = true`, the node will:

1. Create the directory `img_save_path` (via `mkdir -p`)
2. Save an annotated PNG image:

   - Filename includes detected marker IDs: `aruco_<id1>_<id2>...png`
   - If no markers: `aruco_no_markers.png`

3. Overlay text on the image:

   - Human-readable timestamp (system clock)
   - Marker coordinate summary (`X, Y, Z` derived from `tvec` conversion)

> The saved image uses the node’s cached `latest_frame_` which includes drawn markers/axes.

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
| `axis_length`           | double |  `0.03` | Length of drawn axes in the debug image (meters).                                                    |
| `dictionary_id`         |    int |     `1` | OpenCV predefined dictionary enum value (must match the printed markers).                           |
| `min_bounding_box_area` | double | `100.0` | Filters detections: marker’s 2D bounding box area in pixels must be ≥ this threshold to be accepted. |

**Bounding box filtering details**

- For each detected marker’s 4 corner points, the node computes:

  - `min_x, max_x, min_y, max_y`
  - area = `(max_x - min_x) * (max_y - min_y)`

- If area < `min_bounding_box_area`, the marker is ignored (no TF, no output pose).

This helps reject:

- tiny far-away false positives
- noisy corner detections
- partially detected markers

---

### Frames / transforms

| Parameter         |   Type |               Default | Description                                                                                 |
| ----------------- | -----: | --------------------: | ------------------------------------------------------------------------------------------- |
| `camera_frame`    | string | `camera_link_optical` | Frame ID assigned to marker poses before TF transform. Should be the camera optical frame. |
| `tf_output_frame` | string |                `odom` | Target frame to transform marker poses into. Also used as `frame_id` in outputs.            |

---

### Image I/O

| Parameter       |   Type |                          Default | Description                                                              |
| --------------- | -----: | -------------------------------: | ------------------------------------------------------------------------ |
| `input_img`     | string | `/camera/camera/color/image_raw` | Raw image topic (base).                                                  |
| `output_img`    | string |         `/detection/aruco/image` | Output annotated image topic (base).                                     |
| `compressed_io` |   bool |                          `false` | If true: subscribe/publish to `<topic>/compressed` as `CompressedImage`. |
| `publish_img`   |   bool |                           `true` | Publish annotated debug images.                                          |

---

### Outputs

| Parameter        |   Type |                       Default | Description                                                       |
| ---------------- | -----: | ----------------------------: | ----------------------------------------------------------------- |
| `publish_tf`     |   bool |                        `true` | Broadcast TF transforms `aruco_marker_<id>` in `tf_output_frame`. |
| `publish_output` |   bool |                       `false` | Publish `ObjectDetections` messages to `output_topic`.            |
| `output_topic`   | string | `/detection/aruco/detections` | Detection output topic name.                                      |

---

### Camera calibration

You support two modes:

#### 1) Parameter-based calibration (always initialized)

These parameters are always declared and used as initial calibration.

| Parameter                 |      Type |                                   Default | Description                                      |
| ------------------------- | --------: | ----------------------------------------: | ------------------------------------------------ |
| `camera_matrix`           | double[9] | `[530.4, 0, 320, 0, 530.4, 240, 0, 0, 1]` | Row-major intrinsic matrix K.                    |
| `distortion_coefficients` |  double[] |                             `[0,0,0,0,0]` | Distortion coeffs, typically `[k1,k2,p1,p2,k3]`. |

#### 2) CameraInfo override (optional)

If enabled, incoming `CameraInfo` replaces intrinsics.

| Parameter           |   Type |                            Default | Description                                                                            |
| ------------------- | -----: | ---------------------------------: | -------------------------------------------------------------------------------------- |
| `use_camera_info`   |   bool |                            `false` | If true, subscribe to `camera_info_topic` and overwrite camera intrinsics dynamically. |
| `camera_info_topic` | string | `/camera/camera/color/camera_info` | Topic for `sensor_msgs/CameraInfo`.                                                    |

**CameraInfo behavior**

- `camera_matrix_` is built from `msg->k[0..8]`
- `dist_coeffs_` is built from `msg->d[]` (any length supported)

> In `processImage`, if `camera_matrix_` is empty, pose estimation is skipped (warn once).
> In the current code, `camera_matrix_` is initialized from params, so it will not be empty unless changed elsewhere.

---

## Detection Pipeline (detailed)

1. Receive image (raw or compressed)
2. Detect markers with:

   ```cpp
   detector_.detectMarkers(frame, corners, ids);
   ```

3. Clone frame for annotation:

   ```cpp
   annotated_frame = frame.clone();
   ```

4. Clear cached detections and update timestamp:

   - `latest_ids_`, `latest_poses_`
   - `latest_timestamp_ = header.stamp`

5. If markers exist:

   - estimate pose:

     ```cpp
     estimatePoseSingleMarkers(corners, marker_length_, camera_matrix_, dist_coeffs_, rvecs, tvecs);
     ```

   - draw marker borders
   - for each marker:

     - compute bbox area in pixels
     - apply `min_bounding_box_area` filter
     - draw axes
     - convert pose + transform to output frame
     - cache pose + id
     - optionally publish TF transform

6. Cache annotated frame:

   - `latest_frame_ = annotated_frame.clone()`
   - `latest_marker_coords_` stores marker positions (in the camera-converted XYZ convention used for display)

7. Optionally publish `ObjectDetections` message if enabled.

---

## Example YAML Configuration

```yaml
aruco_detector:
  ros__parameters:
    marker_length: 0.35
    axis_length: 0.03
    # 4x4: 50=0, 100=1, 250=2, 1000=3 | 5x5: 50=4, 100=5, 250=6, 1000=7 | 6x6: 50=8, 100=9, 250=10, 1000=11
    dictionary_id: 1
    min_bounding_box_area: 150.0

    camera_frame: camera_link_optical
    tf_output_frame: odom

    input_img: /camera/camera/color/image_raw
    output_img: /detection/aruco/image

    compressed_io: false
    publish_img: true

    publish_tf: true

    publish_output: true
    output_topic: /detection/aruco/detections

    use_camera_info: true
    camera_info_topic: /camera/camera/color/camera_info
```

---

## Usage

### Run with params file

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

- In current modifications, `camera_info` subscription happens only in the raw-image path.
