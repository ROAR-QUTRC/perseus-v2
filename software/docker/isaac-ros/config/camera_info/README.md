# Camera calibration

`default_camera.yaml` is a **placeholder** 640×480 pinhole calibration so the
stack starts. AprilTag and detection **poses are only as accurate as this
file** — replace it with a real calibration for your camera:

```console
ros2 run camera_calibration cameracalibrator \
  --size 8x6 --square 0.025 \
  image:=/image_raw camera:=/camera
```

Save the resulting YAML here (keep the `camera_name`/filename referenced by the
launch file, or update `camera_info_url` in `../perseus_isaac.yaml`). This
directory is mounted read-only into the container at `/opt/perseus/camera_info`.
