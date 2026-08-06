#!/usr/bin/env python3
"""Stream a PCD file to a live ROS 2 PointCloud2 topic.

Written for replaying the maps FAST-LIO saves via the `pcd_save` block of
`autonomy/config/livox_mid360.yaml` (default `~/maps/scan.pcd`) without needing the rover,
the LiDAR, or Gazebo running. Useful for exercising anything downstream of the cloud —
costmap layers, terrain/heightmap experiments, RViz displays — off a recorded map.

FAST-LIO writes with `pcl::PCDWriter::writeBinary` (`laserMapping.cpp:663`), i.e. plain
uncompressed binary PCD, so this parses the format directly with numpy. No PCL, no
pcl_conversions, no new package dependency.

Defaults are set for the common case: read `~/maps/scan.pcd`, publish it once, and latch it
so subscribers that start later still get it. A FAST-LIO map is tens of megabytes, so
republishing one at a sensor-like rate saturates the DDS transport for no benefit — a static
map does not change. Pass `should_loop:=true` to republish continuously.

Two modes, selected by `points_per_message`:

  * 0 (default) — the whole cloud in one message. What you want for a static map: point RViz
    at it, or hand a costmap/heightmap node a complete map.
  * N > 0 — N points per message, walking through the file. Makes a saved map look roughly
    like a live sensor stream, for testing incremental ingestion. This is a walk in file
    order, *not* a real sensor sweep — see "Limitations".

Usage
-----
Publish `~/maps/scan.pcd` once, latched (no arguments needed):

    python3 pcd_publisher.py

A different map, republished continuously at 1 Hz:

    python3 pcd_publisher.py --ros-args \
        -p pcd_file:=$HOME/maps/scan_20260803_082004.pcd \
        -p should_loop:=true -p rate_hz:=1.0

Fake a sensor stream of 20k points per message at 10 Hz:

    python3 pcd_publisher.py --ros-args -p points_per_message:=20000 -p should_loop:=true

Against the sim clock, add `-p use_sim_time:=true` — timestamps come from the node clock, so
that is all it takes.

Parameters
----------
pcd_file            (string, ~/maps/scan.pcd)  Path to the .pcd file. `~` is expanded.
topic               (string, /Laser_map)  Topic to publish on. Deliberately not
                    /livox/lidar, so this cannot be mistaken for the real sensor.
frame_id            (string, map)  frame_id stamped on the message. Must exist in TF for
                    RViz and most consumers to display it.
rate_hz             (double, 10.0)  Timer rate. Only meaningful when republishing or
                    chunking; a single latched publish ignores it after the first tick.
points_per_message  (int, 0)  0 = whole cloud per message; N = N points per message.
should_loop         (bool, false)  Restart from the beginning once the file is exhausted.
                    Left false, the node publishes the cloud once and then idles.
should_latch        (bool, true)  Use TRANSIENT_LOCAL durability so subscribers that join
                    late still receive the last message. Needed for a one-shot publish to be
                    useful at all, and required if a subscriber requests transient_local,
                    since a volatile publisher will not match it.

Limitations
-----------
* `DATA binary_compressed` is not supported — it needs LZF decompression, which is not
  implemented here rather than implemented untested. The node reports this clearly and says
  how to convert. FAST-LIO never writes this format.
* No trajectory or sensor origin. A saved map is just points; the sensor pose that observed
  each one is gone. Anything needing per-ray origins (occupancy ray-casting, free-space
  carving) cannot be driven properly from a PCD alone — replay the rosbag for that.
* Points are emitted in file order. In `points_per_message` mode, consecutive messages are
  spatially clustered, which is not how a real sweep arrives.
* NaN points, if any, are passed through untouched.
"""

import os
import sys
from dataclasses import dataclass

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField

DEFAULT_PCD_FILE = "~/maps/scan.pcd"
DEFAULT_TOPIC = "/Laser_map"
DEFAULT_FRAME_ID = "odom"
DEFAULT_RATE_HZ = 10.0

# PointCloud2 represents an unordered cloud as a single row of `width` points.
UNORDERED_CLOUD_HEIGHT = 1

# Queue depth of one is all a latched publisher needs; a streaming one wants a little slack.
LATCHED_QUEUE_DEPTH = 1
STREAMING_QUEUE_DEPTH = 5

BYTES_PER_MEGABYTE = 1_000_000
# Above roughly ten megabytes a message depends on DDS fragmentation to arrive at all.
LARGE_MESSAGE_WARNING_BYTES = 10 * 1024 * 1024
# Sustained throughput past this is well beyond what a default DDS setup handles gracefully.
HIGH_THROUGHPUT_WARNING_BYTES_PER_SECOND = 50 * 1024 * 1024

# (PCD TYPE letter, SIZE in bytes) -> sensor_msgs/PointField datatype constant.
# PCD also permits I8/U8 (int64/uint64); PointField has no equivalent, so those are rejected.
PCD_TYPE_TO_POINTFIELD = {
    ("I", 1): PointField.INT8,
    ("U", 1): PointField.UINT8,
    ("I", 2): PointField.INT16,
    ("U", 2): PointField.UINT16,
    ("I", 4): PointField.INT32,
    ("U", 4): PointField.UINT32,
    ("F", 4): PointField.FLOAT32,
    ("F", 8): PointField.FLOAT64,
}

# numpy dtype per (TYPE, SIZE), for parsing ASCII payloads.
PCD_TYPE_TO_NUMPY = {
    ("I", 1): "<i1",
    ("U", 1): "<u1",
    ("I", 2): "<i2",
    ("U", 2): "<u2",
    ("I", 4): "<i4",
    ("U", 4): "<u4",
    ("F", 4): "<f4",
    ("F", 8): "<f8",
}

RECOGNIZED_HEADER_KEYS = (
    "VERSION",
    "FIELDS",
    "SIZE",
    "TYPE",
    "COUNT",
    "WIDTH",
    "HEIGHT",
    "VIEWPOINT",
    "POINTS",
    "DATA",
)

REQUIRED_HEADER_KEYS = ("FIELDS", "SIZE", "TYPE", "DATA")

PADDING_FIELD_NAME = "_"


class PcdError(RuntimeError):
    """Raised when a PCD file cannot be parsed or is in an unsupported format."""


@dataclass
class PcdCloud:
    """A parsed PCD file, already in PointCloud2's memory layout.

    fields:      PointField entries for the real (non-padding) fields, at correct offsets.
    point_step:  Bytes per point in `data`. Can exceed the sum of the field sizes when the
                 writer emitted struct padding, which is passed through rather than removed.
    point_count: Number of points.
    data:        Raw payload, exactly `point_count * point_step` bytes.
    """

    fields: list[PointField]
    point_step: int
    point_count: int
    data: bytes


@dataclass
class PublisherSettings:
    """Resolved ROS parameters for one run of the node."""

    pcd_file: str
    topic: str
    frame_id: str
    rate_hz: float
    points_per_message: int
    should_loop: bool
    should_latch: bool


def _parse_header(raw_bytes: bytes) -> tuple[dict[str, list[str]], int]:
    """Parse the PCD header, returning it with the byte offset where the payload starts.

    PCD headers are ASCII lines terminated by DATA; `#` comments and blank lines are skipped.
    """
    header: dict[str, list[str]] = {}
    offset = 0
    has_data_line = False

    while offset < len(raw_bytes):
        newline = raw_bytes.find(b"\n", offset)
        if newline == -1:
            raise PcdError("header ended without a DATA line (truncated file?)")
        line = raw_bytes[offset:newline].decode("ascii", errors="replace").strip()
        offset = newline + 1

        if not line or line.startswith("#"):
            continue

        tokens = line.split()
        key = tokens[0].upper()
        # Unknown keys are tolerated; newer PCD revisions may add them.
        if key in RECOGNIZED_HEADER_KEYS:
            header[key] = tokens[1:]

        if key == "DATA":
            has_data_line = True
            break

    if not has_data_line:
        raise PcdError("no DATA line found in header")

    for required in REQUIRED_HEADER_KEYS:
        if required not in header:
            raise PcdError(f"header is missing required key {required}")

    return header, offset


def _read_field_columns(
    header: dict[str, list[str]],
) -> list[tuple[str, int, str, int]]:
    """Return the header's per-field (name, size, type letter, count) tuples."""
    names = header["FIELDS"]
    sizes = [int(size) for size in header["SIZE"]]
    types = [type_char.upper() for type_char in header["TYPE"]]
    if "COUNT" in header:
        counts = [int(count) for count in header["COUNT"]]
    else:
        counts = [1] * len(names)

    if not (len(names) == len(sizes) == len(types) == len(counts)):
        raise PcdError(
            "FIELDS/SIZE/TYPE/COUNT lengths disagree "
            f"({len(names)}/{len(sizes)}/{len(types)}/{len(counts)})"
        )

    return list(zip(names, sizes, types, counts))


def _to_pointfield_datatype(name: str, type_char: str, size: int) -> int:
    """Map a PCD type letter and size to a PointField datatype, or raise."""
    key = (type_char, size)
    if key not in PCD_TYPE_TO_POINTFIELD:
        raise PcdError(
            f"field '{name}' has unsupported type/size {type_char}{size}; "
            "PointCloud2 has no matching datatype"
        )
    return PCD_TYPE_TO_POINTFIELD[key]


def _build_field_layout(header: dict[str, list[str]]) -> tuple[list[PointField], int]:
    """Build PointField entries from the header, plus the total per-point stride.

    Some PCL versions represent struct padding as fake fields named `_`, so that the declared
    fields cover sizeof(PointT) exactly; others pack the fields tightly and emit none. Both
    are handled: offsets accumulate over every declared field, but `_` entries are left out
    of the output. That is what lets the payload be republished as-is, without repacking.
    """
    fields: list[PointField] = []
    offset = 0

    for name, size, type_char, count in _read_field_columns(header):
        if name != PADDING_FIELD_NAME:
            fields.append(
                PointField(
                    name=name,
                    offset=offset,
                    datatype=_to_pointfield_datatype(name, type_char, size),
                    count=count,
                )
            )
        offset += size * count

    if not fields:
        raise PcdError("no usable fields (all padding?)")

    return fields, offset


def _resolve_point_count(
    header: dict[str, list[str]], point_step: int, payload_length: int
) -> int:
    """Resolve the point count from POINTS, or WIDTH*HEIGHT, or the payload size."""
    if "POINTS" in header:
        return int(header["POINTS"][0])
    if "WIDTH" in header and "HEIGHT" in header:
        return int(header["WIDTH"][0]) * int(header["HEIGHT"][0])
    if point_step:
        return payload_length // point_step
    raise PcdError("cannot determine the number of points")


def _read_ascii_payload(
    payload: bytes, header: dict[str, list[str]], point_count: int
) -> PcdCloud:
    """Parse an ASCII payload into a tightly packed binary blob.

    PCL's ASCII writer emits only real fields, never `_` padding, so offsets are recomputed
    packed rather than passed through as they are for binary payloads.
    """
    kept_fields = [
        column
        for column in _read_field_columns(header)
        if column[0] != PADDING_FIELD_NAME
    ]
    if any(count != 1 for _, _, _, count in kept_fields):
        raise PcdError(
            "ASCII PCD with multi-element fields (COUNT > 1) is not supported"
        )

    # np.fromstring(..., sep=" ") would also work, but it is deprecated and gone in numpy 2.
    values = np.array(payload.split(), dtype=np.float64)
    expected_value_count = point_count * len(kept_fields)
    if values.size < expected_value_count:
        raise PcdError(
            f"ASCII payload has {values.size} values, expected {expected_value_count} "
            f"({point_count} points x {len(kept_fields)} fields)"
        )
    table = values[:expected_value_count].reshape(point_count, len(kept_fields))

    dtype = np.dtype(
        {
            "names": [name for name, _, _, _ in kept_fields],
            "formats": [
                PCD_TYPE_TO_NUMPY[(type_char, size)]
                for _, size, type_char, _ in kept_fields
            ],
        }
    )
    record = np.empty(point_count, dtype=dtype)
    for column_index, (name, _, _, _) in enumerate(kept_fields):
        record[name] = table[:, column_index]

    fields = [
        PointField(
            name=name,
            offset=dtype.fields[name][1],
            datatype=_to_pointfield_datatype(name, type_char, size),
            count=1,
        )
        for name, size, type_char, _ in kept_fields
    ]

    return PcdCloud(
        fields=fields,
        point_step=dtype.itemsize,
        point_count=point_count,
        data=record.tobytes(),
    )


def read_pcd(path: str) -> PcdCloud:
    """Read a PCD file into a PointCloud2-ready layout.

    Supports `DATA ascii` and `DATA binary`. Raises PcdError on anything else, on a malformed
    header, or on a truncated payload.
    """
    with open(path, "rb") as handle:
        raw_bytes = handle.read()

    if not raw_bytes:
        raise PcdError("file is empty")

    header, payload_start = _parse_header(raw_bytes)
    data_format = header["DATA"][0].lower()
    payload = raw_bytes[payload_start:]

    if data_format == "binary_compressed":
        raise PcdError(
            "DATA binary_compressed is not supported. Convert it first, e.g.\n"
            "  pcl_convert_pcd_ascii_binary in.pcd out.pcd 1\n"
            "(the trailing 1 selects uncompressed binary)"
        )

    fields, point_step = _build_field_layout(header)
    point_count = _resolve_point_count(header, point_step, len(payload))

    if point_count == 0:
        raise PcdError("file declares zero points")

    if data_format == "ascii":
        return _read_ascii_payload(payload, header, point_count)

    if data_format != "binary":
        raise PcdError(f"unsupported DATA format '{data_format}'")

    needed_bytes = point_count * point_step
    if len(payload) < needed_bytes:
        raise PcdError(
            f"payload is truncated: need {needed_bytes} bytes for {point_count} points "
            f"at {point_step} B/point, found {len(payload)}"
        )

    return PcdCloud(
        fields=fields,
        point_step=point_step,
        point_count=point_count,
        data=payload[:needed_bytes],
    )


class PcdPublisher(Node):
    """Publishes a PCD file to a PointCloud2 topic, whole or in chunks."""

    def __init__(self):
        super().__init__("pcd_publisher")

        settings = self._declare_settings()
        self._frame_id = settings.frame_id
        self._points_per_message = settings.points_per_message
        self._should_loop = settings.should_loop

        self.get_logger().info(f"Reading {settings.pcd_file}")
        self._cloud = read_pcd(settings.pcd_file)
        self._log_cloud_summary()
        self._warn_about_throughput(settings)

        self._publisher = self._create_cloud_publisher(settings)
        self._next_point_index = 0
        self._is_finished = False
        self._timer = self.create_timer(
            1.0 / settings.rate_hz, self._publish_next_message
        )
        self._log_configuration(settings)

    def _declare_settings(self) -> PublisherSettings:
        """Declare and validate every ROS parameter this node takes."""
        settings = PublisherSettings(
            pcd_file=os.path.expanduser(
                self.declare_parameter("pcd_file", DEFAULT_PCD_FILE).value
            ),
            topic=self.declare_parameter("topic", DEFAULT_TOPIC).value,
            frame_id=self.declare_parameter("frame_id", DEFAULT_FRAME_ID).value,
            rate_hz=self.declare_parameter("rate_hz", DEFAULT_RATE_HZ).value,
            points_per_message=self.declare_parameter("points_per_message", 0).value,
            should_loop=self.declare_parameter("should_loop", False).value,
            should_latch=self.declare_parameter("should_latch", True).value,
        )

        if not settings.pcd_file:
            raise PcdError("pcd_file cannot be empty")
        if settings.rate_hz <= 0.0:
            raise PcdError(f"rate_hz must be positive, got {settings.rate_hz}")
        if settings.points_per_message < 0:
            raise PcdError(
                "points_per_message cannot be negative, got "
                f"{settings.points_per_message}"
            )

        return settings

    def _create_cloud_publisher(self, settings: PublisherSettings):
        """Create the PointCloud2 publisher with QoS matching the requested behavior.

        Reliable satisfies both reliable and best-effort subscribers, so it is the safer
        default. Durability is the setting that genuinely fails to match: a volatile
        publisher is invisible to a transient_local subscriber, hence should_latch.
        """
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=(
                LATCHED_QUEUE_DEPTH if settings.should_latch else STREAMING_QUEUE_DEPTH
            ),
            durability=(
                DurabilityPolicy.TRANSIENT_LOCAL
                if settings.should_latch
                else DurabilityPolicy.VOLATILE
            ),
        )
        return self.create_publisher(PointCloud2, settings.topic, qos)

    def _log_cloud_summary(self):
        """Log what was loaded, so a wrong file is obvious immediately."""
        field_names = ", ".join(field.name for field in self._cloud.fields)
        self.get_logger().info(
            f"Loaded {self._cloud.point_count} points, "
            f"{self._cloud.point_step} B/point "
            f"[{field_names}]"
        )

    def _log_configuration(self, settings: PublisherSettings):
        """Log the resolved publishing behavior."""
        if self._points_per_message == 0:
            mode = "whole cloud per message"
        else:
            mode = f"{self._points_per_message} points per message"
        self.get_logger().info(
            f"Publishing on {settings.topic} in frame '{self._frame_id}' at "
            f"{settings.rate_hz} Hz ({mode}, should_loop={self._should_loop}, "
            f"should_latch={settings.should_latch})"
        )

    def _warn_about_throughput(self, settings: PublisherSettings):
        """Warn when the configured rate would overwhelm the transport.

        A FAST-LIO map is easily tens of megabytes, and republishing one at a sensor-like
        rate saturates DDS long before it saturates the network. This warns rather than
        clamps: only the caller knows what the consumer can take.
        """
        points_per_message = self._points_per_message or self._cloud.point_count
        bytes_per_message = points_per_message * self._cloud.point_step

        if bytes_per_message > LARGE_MESSAGE_WARNING_BYTES:
            self.get_logger().warning(
                f"Each message is {bytes_per_message / BYTES_PER_MEGABYTE:.1f} MB. "
                "Messages this large rely on DDS fragmentation and may be dropped "
                "outright; consider points_per_message."
            )

        if not settings.should_loop and self._points_per_message == 0:
            # Published once, so the rate never applies to a second message.
            return

        bytes_per_second = bytes_per_message * settings.rate_hz
        if bytes_per_second > HIGH_THROUGHPUT_WARNING_BYTES_PER_SECOND:
            self.get_logger().warning(
                f"That is {bytes_per_second / BYTES_PER_MEGABYTE:.0f} MB/s at "
                f"{settings.rate_hz} Hz. Lower rate_hz, or leave should_loop false to "
                "publish a static map just once."
            )

    def _build_message(self, start_index: int, point_count: int) -> PointCloud2:
        """Slice `point_count` points starting at `start_index` into a PointCloud2."""
        point_step = self._cloud.point_step
        message = PointCloud2()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self._frame_id
        message.height = UNORDERED_CLOUD_HEIGHT
        message.width = point_count
        message.fields = self._cloud.fields
        message.is_bigendian = False
        message.point_step = point_step
        message.row_step = point_step * point_count
        message.data = self._cloud.data[
            start_index * point_step : (start_index + point_count) * point_step
        ]
        # Only claimable if the cloud is known finite, and NaNs are passed through untouched.
        message.is_dense = False
        return message

    def _publish_next_message(self):
        """Publish the next message, advancing through the cloud when chunking."""
        if self._is_finished:
            return

        total_points = self._cloud.point_count

        if self._points_per_message == 0:
            self._publisher.publish(self._build_message(0, total_points))
            if not self._should_loop:
                self.get_logger().info(
                    "Published the whole cloud once; idling so latched subscribers can "
                    "still receive it. Set should_loop:=true to republish."
                )
                self._is_finished = True
            return

        remaining = total_points - self._next_point_index
        point_count = min(self._points_per_message, remaining)
        self._publisher.publish(
            self._build_message(self._next_point_index, point_count)
        )
        self._next_point_index += point_count

        if self._next_point_index < total_points:
            return

        if self._should_loop:
            self._next_point_index = 0
        else:
            self.get_logger().info(
                f"Reached the end of the cloud after {total_points} points; idling."
            )
            self._is_finished = True


def main(argv: list[str] | None = None) -> int:
    rclpy.init(args=argv)
    node = None
    try:
        node = PcdPublisher()
        rclpy.spin(node)
    except PcdError as error:
        # Configuration and file problems are the expected failures; report them plainly
        # rather than as a traceback.
        print(f"pcd_publisher: {error}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
