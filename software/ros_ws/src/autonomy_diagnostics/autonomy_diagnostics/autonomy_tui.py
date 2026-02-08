#!/usr/bin/env python3
"""
Autonomy Diagnostics TUI - A comprehensive diagnostic tool for autonomy system monitoring.

Uses curses (standard library) for the terminal interface.
Requires PyYAML for configuration loading.
Falls back to simple text mode when no TTY is available.

This tool provides a real-time view of:
- Topic presence and rates (scan, imu, odom, etc.)
- TF frame connectivity (map->odom->base_link chain)
- Nav2 lifecycle node states
- Config file existence

Configuration is loaded from config/diagnostics.yaml in the package share directory.
"""

import argparse
import curses
import glob as glob_module
import logging
import math
import os
import signal
import socket
import subprocess
import sys
import termios
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Any, Dict, List, Optional

import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import Imu, JointState, Joy, LaserScan
from tf2_ros import Buffer, TransformListener
from tf2_ros import ConnectivityException, ExtrapolationException, LookupException

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

# =============================================================================
# Constants
# =============================================================================

# Timing thresholds (seconds)
STALE_DATA_TIMEOUT = 2.0
TF_CHECK_INTERVAL = 1.0
LIFECYCLE_CHECK_INTERVAL = 2.0

# Rate calculation
RATE_WINDOW_SIZE = 10

# UI refresh
CURSES_REFRESH_MS = 200
SIMPLE_MODE_REFRESH_INTERVAL = 1.0

# Rate thresholds for status
RATE_WARNING_THRESHOLD = 0.8  # Warn if rate < 80% expected
RATE_CRITICAL_THRESHOLD = 0.5  # Critical if rate < 50% expected


def get_ros_ws_src_path() -> str:
    """Get the ROS workspace src path dynamically.

    Attempts to find the path by:
    1. Looking for a git repository root and finding ros_ws/src within it
    2. Falling back to ~/perseus-v2/software/ros_ws/src
    """
    try:
        # Try to find git repo root
        result = subprocess.run(
            ["git", "rev-parse", "--show-toplevel"],
            capture_output=True,
            text=True,
            timeout=5,
        )
        if result.returncode == 0:
            git_root = result.stdout.strip()
            ros_ws_src = os.path.join(git_root, "software", "ros_ws", "src")
            if os.path.isdir(ros_ws_src):
                return ros_ws_src
    except (subprocess.TimeoutExpired, FileNotFoundError):
        pass

    # Fallback to home directory based path
    home_path = os.path.expanduser("~/perseus-v2/software/ros_ws/src")
    if os.path.isdir(home_path):
        return home_path

    # Last resort: return empty string (config file checks will fail gracefully)
    return ""


# =============================================================================
# Configuration Loading
# =============================================================================


def load_config() -> Dict[str, Any]:
    """Load configuration from the package's config file.

    Returns a dictionary with keys: monitored_topics, tf_frames, lifecycle_nodes,
    config_files. Falls back to defaults if config file is not found.
    """
    # Default configuration (fallback if config file not found)
    defaults = {
        "settings": {
            "stale_data_timeout": STALE_DATA_TIMEOUT,
            "tf_check_interval": TF_CHECK_INTERVAL,
            "lifecycle_check_interval": LIFECYCLE_CHECK_INTERVAL,
            "rate_window_size": RATE_WINDOW_SIZE,
            "rate_warning_threshold": RATE_WARNING_THRESHOLD,
            "rate_critical_threshold": RATE_CRITICAL_THRESHOLD,
            "refresh_ms": CURSES_REFRESH_MS,
            "refresh_min_ms": 100,
            "refresh_max_ms": 2000,
            "refresh_step_ms": 100,
        },
        "monitored_topics": [
            {
                "name": "scan",
                "topic": "/scan",
                "type": "LaserScan",
                "expected_hz": 10.0,
                "critical": True,
            },
            {
                "name": "imu",
                "topic": "/imu/data",
                "type": "Imu",
                "expected_hz": 100.0,
                "critical": True,
            },
            {
                "name": "odom",
                "topic": "/odom",
                "type": "Odometry",
                "expected_hz": 50.0,
                "critical": True,
            },
            {
                "name": "odom_filtered",
                "topic": "/odometry/filtered",
                "type": "Odometry",
                "expected_hz": 30.0,
                "critical": True,
            },
            {
                "name": "map",
                "topic": "/map",
                "type": "OccupancyGrid",
                "expected_hz": 0.1,
                "critical": False,
            },
            {
                "name": "cmd_vel",
                "topic": "/cmd_vel",
                "type": "Twist",
                "expected_hz": 0.0,
                "critical": False,
            },
            {
                "name": "joint_states",
                "topic": "/joint_states",
                "type": "JointState",
                "expected_hz": 50.0,
                "critical": True,
            },
        ],
        "tf_frames": [
            {
                "parent": "map",
                "child": "odom",
                "description": "SLAM/AMCL",
                "critical": True,
            },
            {
                "parent": "odom",
                "child": "base_link",
                "description": "EKF",
                "critical": True,
            },
            {
                "parent": "base_link",
                "child": "chassis",
                "description": "URDF",
                "critical": True,
            },
            {
                "parent": "chassis",
                "child": "laser_2d_frame",
                "description": "LiDAR",
                "critical": True,
            },
            {
                "parent": "chassis",
                "child": "imu_link",
                "description": "IMU",
                "critical": True,
            },
        ],
        "lifecycle_nodes": [
            "bt_navigator",
            "controller_server",
            "planner_server",
            "map_server",
            "amcl",
            "smoother_server",
            "velocity_smoother",
            "collision_monitor",
            "waypoint_follower",
            "behavior_server",
        ],
        "config_files": [
            {"path": "autonomy/config/ekf_params.yaml", "description": "EKF params"},
            {
                "path": "autonomy/config/slam_toolbox_params.yaml",
                "description": "SLAM params",
            },
            {
                "path": "autonomy/config/perseus_nav_params.yaml",
                "description": "Nav2 params",
            },
            {"path": "autonomy/config/cmd_vel_mux.yaml", "description": "Mux config"},
        ],
        "joystick": {
            "device_path": "/dev/input/js*",
        },
    }

    logger = logging.getLogger(__name__)
    try:
        pkg_share = get_package_share_directory("autonomy_diagnostics")
        config_path = os.path.join(pkg_share, "config", "diagnostics.yaml")

        if os.path.isfile(config_path):
            with open(config_path, "r") as f:
                config = yaml.safe_load(f)
                if config:
                    # Merge with defaults for any missing keys
                    for key in defaults:
                        if key not in config:
                            config[key] = defaults[key]
                    return config
    except Exception as e:
        logger.warning(f"Failed to load config file, using defaults: {e}")

    return defaults


# =============================================================================
# Data Classes
# =============================================================================


@dataclass
class TopicStatus:
    """Status of a monitored topic."""

    name: str
    topic: str
    msg_type: str
    expected_hz: float
    actual_hz: float = 0.0
    last_msg_time: float = 0.0
    msg_count: int = 0
    critical: bool = True
    stale_data_timeout: float = STALE_DATA_TIMEOUT
    rate_warning_threshold: float = RATE_WARNING_THRESHOLD
    rate_critical_threshold: float = RATE_CRITICAL_THRESHOLD

    @property
    def status(self) -> str:
        """Determine status based on rate and staleness."""
        now = time.time()
        if self.msg_count == 0:
            return "NONE"
        if now - self.last_msg_time > self.stale_data_timeout:
            return "STALE"
        if self.expected_hz <= 0:
            return "OK"  # Variable rate topics
        ratio = self.actual_hz / self.expected_hz
        if ratio < self.rate_critical_threshold:
            return "CRIT"
        if ratio < self.rate_warning_threshold:
            return "WARN"
        return "OK"


@dataclass
class TFStatus:
    """Status of a TF transform."""

    parent: str
    child: str
    description: str
    connected: bool = False
    error_msg: str = ""
    critical: bool = True


@dataclass
class LifecycleStatus:
    """Status of a lifecycle node."""

    name: str
    state: str = "unknown"
    last_check: float = 0.0


@dataclass
class ConfigStatus:
    """Status of a config file."""

    path: str
    description: str
    exists: bool = False


@dataclass
class JoystickStatus:
    """Status of a joystick device."""

    device_path: str = ""
    device_connected: bool = False
    num_axes: int = 0
    num_buttons: int = 0
    last_msg_time: float = 0.0


@dataclass
class LidarScanData:
    """Cached laser scan data for visualization."""

    angle_min: float = 0.0
    angle_max: float = 0.0
    angle_increment: float = 0.0
    range_min: float = 0.0
    range_max: float = 0.0
    ranges: Optional[List[float]] = None
    timestamp: float = 0.0


# =============================================================================
# Rate Calculator
# =============================================================================


class RateCalculator:
    """Calculate message rate from timestamps using sliding window."""

    def __init__(self, window_size: int = RATE_WINDOW_SIZE):
        self.timestamps: deque = deque(maxlen=window_size)

    def add_timestamp(self, ts: float) -> float:
        self.timestamps.append(ts)
        if len(self.timestamps) < 2:
            return 0.0
        duration = self.timestamps[-1] - self.timestamps[0]
        if duration <= 0:
            return 0.0
        return (len(self.timestamps) - 1) / duration


# =============================================================================
# Main Node
# =============================================================================


class AutonomyDiagnosticsNode(Node):
    """ROS2 node for autonomy system diagnostics."""

    def __init__(self):
        super().__init__("autonomy_diagnostics")

        # Load configuration from YAML file
        self.config = load_config()
        self.ros_ws_src_path = get_ros_ws_src_path()

        # Extract settings from config
        settings = self.config["settings"]
        self.stale_data_timeout = settings["stale_data_timeout"]
        self.tf_check_interval = settings["tf_check_interval"]
        self.lifecycle_check_interval = settings["lifecycle_check_interval"]
        self.rate_window_size = settings["rate_window_size"]
        self.rate_warning_threshold = settings["rate_warning_threshold"]
        self.rate_critical_threshold = settings["rate_critical_threshold"]

        # Callback group for service calls
        self.cb_group = ReentrantCallbackGroup()

        # Lock for thread-safe access to status data
        self._status_lock = threading.Lock()

        # Initialize data structures
        self.topic_statuses: Dict[str, TopicStatus] = {}
        self.rate_calculators: Dict[str, RateCalculator] = {}
        self.tf_statuses: List[TFStatus] = []
        self.lifecycle_statuses: Dict[str, LifecycleStatus] = {}
        self.config_statuses: List[ConfigStatus] = []
        self.joystick_status = JoystickStatus()
        self.lidar_scan = LidarScanData()

        # Lifecycle service clients (created once, reused)
        self._lifecycle_clients: Dict[str, Any] = {}
        self._pending_lifecycle_futures: Dict[str, Any] = {}

        # TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Initialize monitoring systems
        self._setup_topic_monitoring()

        # Initialize TF status tracking
        self._setup_tf_monitoring()

        # Initialize lifecycle node tracking
        self._setup_lifecycle_monitoring()

        # Check config files and joystick device once at startup
        self._check_config_files()
        self._check_joystick_device()

        # Create timers for periodic checks
        self.tf_timer = self.create_timer(
            self.tf_check_interval, self._check_tf_frames, callback_group=self.cb_group
        )
        self.lifecycle_timer = self.create_timer(
            self.lifecycle_check_interval,
            self._check_lifecycle_nodes,
            callback_group=self.cb_group,
        )

        self.get_logger().info("Autonomy diagnostics node started")

    def _setup_topic_monitoring(self):
        """Set up subscribers for all monitored topics.

        Note: Subscribing to topics like LaserScan does add some bandwidth overhead,
        but the callbacks only record timestamps for rate calculation - the actual
        message data is not stored or processed, minimizing memory impact.
        """
        # QoS for sensor data
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )

        for topic_cfg in self.config["monitored_topics"]:
            name = topic_cfg["name"]
            topic = topic_cfg["topic"]
            msg_type = topic_cfg["type"]

            # Create status tracker
            self.topic_statuses[name] = TopicStatus(
                name=name,
                topic=topic,
                msg_type=msg_type,
                expected_hz=topic_cfg["expected_hz"],
                critical=topic_cfg["critical"],
                stale_data_timeout=self.stale_data_timeout,
                rate_warning_threshold=self.rate_warning_threshold,
                rate_critical_threshold=self.rate_critical_threshold,
            )
            self.rate_calculators[name] = RateCalculator(
                window_size=self.rate_window_size
            )

            # Create subscriber based on message type
            if msg_type == "LaserScan":
                self.create_subscription(
                    LaserScan,
                    topic,
                    lambda msg, n=name: self._scan_callback(msg, n),
                    sensor_qos,
                )
            elif msg_type == "Imu":
                self.create_subscription(
                    Imu, topic, lambda msg, n=name: self._topic_callback(n), sensor_qos
                )
            elif msg_type == "Odometry":
                self.create_subscription(
                    Odometry,
                    topic,
                    lambda msg, n=name: self._topic_callback(n),
                    sensor_qos,
                )
            elif msg_type == "OccupancyGrid":
                self.create_subscription(
                    OccupancyGrid,
                    topic,
                    lambda msg, n=name: self._topic_callback(n),
                    10,
                )
            elif msg_type == "Twist":
                self.create_subscription(
                    Twist, topic, lambda msg, n=name: self._topic_callback(n), 10
                )
            elif msg_type == "Joy":
                self.create_subscription(
                    Joy,
                    topic,
                    lambda msg, n=name: self._joy_callback(msg, n),
                    sensor_qos,
                )
            elif msg_type == "JointState":
                self.create_subscription(
                    JointState,
                    topic,
                    lambda msg, n=name: self._topic_callback(n),
                    sensor_qos,
                )

    def _topic_callback(self, topic_name: str):
        """Generic callback for topic monitoring."""
        now = time.time()
        with self._status_lock:
            status = self.topic_statuses[topic_name]
            status.last_msg_time = now
            status.msg_count += 1
            status.actual_hz = self.rate_calculators[topic_name].add_timestamp(now)

    def _joy_callback(self, msg, topic_name: str):
        """Callback for Joy topic - records rate and captures axes/buttons."""
        self._topic_callback(topic_name)
        with self._status_lock:
            self.joystick_status.num_axes = len(msg.axes)
            self.joystick_status.num_buttons = len(msg.buttons)
            self.joystick_status.last_msg_time = time.time()

    def _scan_callback(self, msg, topic_name: str):
        """Callback for LaserScan topic - records rate and stores scan data."""
        self._topic_callback(topic_name)
        with self._status_lock:
            self.lidar_scan.ranges = list(msg.ranges)
            self.lidar_scan.angle_min = msg.angle_min
            self.lidar_scan.angle_max = msg.angle_max
            self.lidar_scan.angle_increment = msg.angle_increment
            self.lidar_scan.range_min = msg.range_min
            self.lidar_scan.range_max = msg.range_max
            self.lidar_scan.timestamp = time.time()

    def _check_joystick_device(self):
        """Check if a joystick device is physically connected."""
        pattern = self.config.get("joystick", {}).get("device_path", "/dev/input/js*")
        devices = sorted(glob_module.glob(pattern))
        with self._status_lock:
            if devices:
                self.joystick_status.device_path = devices[0]
                self.joystick_status.device_connected = True
            else:
                self.joystick_status.device_path = ""
                self.joystick_status.device_connected = False

    def _setup_tf_monitoring(self):
        """Initialize TF status tracking."""
        for tf_cfg in self.config["tf_frames"]:
            self.tf_statuses.append(
                TFStatus(
                    parent=tf_cfg["parent"],
                    child=tf_cfg["child"],
                    description=tf_cfg["description"],
                    critical=tf_cfg["critical"],
                )
            )

    def _check_tf_frames(self):
        """Check connectivity of all required TF frames."""
        # Check if robot_state_publisher is running (publishes static transforms)
        node_names = [name for name, ns in self.get_node_names_and_namespaces()]
        rsp_running = "robot_state_publisher" in node_names

        with self._status_lock:
            for tf_status in self.tf_statuses:
                try:
                    transform = self.tf_buffer.lookup_transform(
                        tf_status.parent,
                        tf_status.child,
                        rclpy.time.Time(),
                        timeout=rclpy.duration.Duration(seconds=0.1),
                    )
                    # Check if transform is stale by comparing timestamp age
                    tf_time_sec = (
                        transform.header.stamp.sec
                        + transform.header.stamp.nanosec * 1e-9
                    )
                    if tf_time_sec == 0:
                        # Static transform - check if robot_state_publisher is running
                        if not rsp_running:
                            tf_status.connected = False
                            tf_status.error_msg = "RSP not running"
                        else:
                            tf_status.connected = True
                            tf_status.error_msg = ""
                    else:
                        # Dynamic transform - check staleness based on transform timestamp
                        now_msg = self.get_clock().now().to_msg()
                        now_sec = now_msg.sec + now_msg.nanosec * 1e-9
                        age = now_sec - tf_time_sec

                        if age > self.stale_data_timeout:
                            tf_status.connected = False
                            tf_status.error_msg = f"stale ({age:.1f}s old)"
                        else:
                            tf_status.connected = True
                            tf_status.error_msg = ""
                except (
                    LookupException,
                    ConnectivityException,
                    ExtrapolationException,
                ) as e:
                    tf_status.connected = False
                    tf_status.error_msg = str(e)[:50]

        # Piggyback joystick device check on the TF timer
        self._check_joystick_device()

    def _setup_lifecycle_monitoring(self):
        """Initialize lifecycle node tracking."""
        for node_name in self.config["lifecycle_nodes"]:
            self.lifecycle_statuses[node_name] = LifecycleStatus(name=node_name)

    def _check_lifecycle_nodes(self):
        """Query lifecycle state of Nav2 nodes.

        Uses persistent service clients and properly handles async responses.
        Checks are non-blocking - results are processed on the next timer cycle.
        """
        available_services = [s[0] for s in self.get_service_names_and_types()]

        for node_name in self.lifecycle_statuses:
            service_name = f"/{node_name}/get_state"

            # First, check if we have a pending future from a previous call
            if node_name in self._pending_lifecycle_futures:
                future = self._pending_lifecycle_futures[node_name]
                if future.done():
                    try:
                        result = future.result()
                        with self._status_lock:
                            if result is not None:
                                self.lifecycle_statuses[
                                    node_name
                                ].state = result.current_state.label
                            else:
                                self.lifecycle_statuses[node_name].state = "error"
                            self.lifecycle_statuses[node_name].last_check = time.time()
                    except Exception as e:
                        with self._status_lock:
                            self.lifecycle_statuses[node_name].state = "error"
                        self.get_logger().debug(
                            f"Lifecycle result failed for {node_name}: {e}"
                        )
                    del self._pending_lifecycle_futures[node_name]
                # If not done, skip this node - still waiting for response
                continue

            # Check if service exists
            if service_name not in available_services:
                with self._status_lock:
                    self.lifecycle_statuses[node_name].state = "not_found"
                continue

            try:
                # Get or create client for this service
                if node_name not in self._lifecycle_clients:
                    self._lifecycle_clients[node_name] = self.create_client(
                        GetState, service_name, callback_group=self.cb_group
                    )

                client = self._lifecycle_clients[node_name]

                # Check if service is available (non-blocking)
                if not client.service_is_ready():
                    with self._status_lock:
                        self.lifecycle_statuses[node_name].state = "unavail"
                    continue

                # Issue async call and store future for next cycle
                future = client.call_async(GetState.Request())
                self._pending_lifecycle_futures[node_name] = future

            except Exception as e:
                with self._status_lock:
                    self.lifecycle_statuses[node_name].state = "error"
                self.get_logger().debug(f"Lifecycle check failed for {node_name}: {e}")

    def _check_config_files(self):
        """Verify existence of required config files."""
        for cfg in self.config["config_files"]:
            full_path = os.path.join(self.ros_ws_src_path, cfg["path"])
            self.config_statuses.append(
                ConfigStatus(
                    path=cfg["path"],
                    description=cfg["description"],
                    exists=os.path.isfile(full_path),
                )
            )


# =============================================================================
# TUI
# =============================================================================


class AutonomyTUI:
    """Curses-based TUI for autonomy diagnostics."""

    # Color pair indices — dashboard
    COLOR_OK = 1
    COLOR_WARN = 2
    COLOR_CRIT = 3
    COLOR_INFO = 4
    # Color pair indices — YAML editor syntax highlighting
    COLOR_YAML_KEY = 5
    COLOR_YAML_VALUE = 6
    COLOR_YAML_COMMENT = 7
    COLOR_YAML_STRING = 8
    COLOR_YAML_BOOL = 9
    COLOR_YAML_NUMBER = 10
    # Color pair indices — LiDAR view
    COLOR_LIDAR_SCAN = 11
    COLOR_LIDAR_GRID = 12

    def __init__(self, node: AutonomyDiagnosticsNode):
        self.node = node
        self.running = True
        self.stdscr = None
        self.new_domain_id: Optional[int] = None  # Set when user requests domain change
        self.show_help = False  # Toggle for help dialog overlay
        self.show_network = False  # Toggle for network info overlay
        self.show_lidar = False  # Toggle for LiDAR scan view
        self._net_ifaces: List[Dict[str, str]] = []  # Cached interface list
        self._net_sel = 0  # Currently selected interface index
        self._net_neighbors: List[Dict[str, str]] = []  # Cached neighbor list
        self._net_last_sel = -1  # Track selection changes to refresh neighbors
        self._net_neighbor_scroll = 0  # Scroll offset for neighbor list

        # UI refresh settings from config
        settings = node.config["settings"]
        self.refresh_ms = settings["refresh_ms"]
        self.REFRESH_MIN_MS = settings["refresh_min_ms"]
        self.REFRESH_MAX_MS = settings["refresh_max_ms"]
        self.REFRESH_STEP_MS = settings["refresh_step_ms"]

    def safe_addstr(self, y: int, x: int, text: str, attr=0):
        """Safely add string, handling screen boundaries."""
        if self.stdscr is None:
            return
        max_y, max_x = self.stdscr.getmaxyx()
        if y < 0 or y >= max_y or x < 0:
            return
        available = max_x - x - 1
        if available <= 0:
            return
        try:
            self.stdscr.addstr(y, x, text[:available], attr)
        except curses.error:
            pass

    def draw_box(self, y: int, x: int, h: int, w: int, title: str = ""):
        """Draw a box with optional title using ASCII characters."""
        # Top border
        self.safe_addstr(y, x, "+" + "-" * (w - 2) + "+")
        # Title
        if title:
            title_str = f" {title} "
            self.safe_addstr(y, x + 2, title_str, curses.A_BOLD)
        # Sides
        for i in range(1, h - 1):
            self.safe_addstr(y + i, x, "|")
            self.safe_addstr(y + i, x + w - 1, "|")
        # Bottom border
        self.safe_addstr(y + h - 1, x, "+" + "-" * (w - 2) + "+")

    def get_status_attr(self, status: str) -> int:
        """Get curses attribute for status string."""
        if status in ("OK", "active"):
            return curses.color_pair(self.COLOR_OK) | curses.A_BOLD
        elif status in ("WARN", "inactive", "unconfigured"):
            return curses.color_pair(self.COLOR_WARN) | curses.A_BOLD
        elif status in (
            "CRIT",
            "STALE",
            "NONE",
            "not_found",
            "unavail",
            "error",
            "finalized",
        ):
            return curses.color_pair(self.COLOR_CRIT) | curses.A_BOLD
        return curses.A_DIM

    def draw_summary_bar(self, y: int, max_x: int):
        """Draw summary status bar."""
        # Count statuses (thread-safe read)
        with self.node._status_lock:
            topics_ok = sum(
                1 for t in self.node.topic_statuses.values() if t.status == "OK"
            )
            topics_total = len(self.node.topic_statuses)

            tf_ok = sum(1 for t in self.node.tf_statuses if t.connected)
            tf_total = len(self.node.tf_statuses)

            lifecycle_active = sum(
                1 for n in self.node.lifecycle_statuses.values() if n.state == "active"
            )
            lifecycle_total = len(self.node.lifecycle_statuses)

            config_ok = sum(1 for c in self.node.config_statuses if c.exists)
            config_total = len(self.node.config_statuses)

        # Determine overall status
        all_ok = (
            topics_ok == topics_total
            and tf_ok == tf_total
            and config_ok == config_total
        )

        summary = f" SUMMARY: Topics {topics_ok}/{topics_total}"
        summary += f" | TF {tf_ok}/{tf_total}"
        summary += f" | Lifecycle {lifecycle_active}/{lifecycle_total}"
        summary += f" | Config {config_ok}/{config_total}"
        summary += " "

        attr = (
            curses.color_pair(self.COLOR_OK)
            if all_ok
            else curses.color_pair(self.COLOR_WARN)
        )
        self.safe_addstr(y, 0, summary.ljust(max_x), attr | curses.A_BOLD)

    def draw_topics_panel(self, y: int, x: int, w: int, h: int):
        """Draw topics monitoring panel."""
        self.draw_box(y, x, h, w, "TOPICS")

        row = y + 1
        # Header
        header = f"{'Topic':<22} {'Rate':>8} {'Exp':>6} {'Status':>6}"
        self.safe_addstr(row, x + 2, header, curses.A_BOLD)
        row += 1
        self.safe_addstr(row, x + 2, "-" * (w - 4))
        row += 1

        # Thread-safe snapshot of topic statuses
        with self.node._status_lock:
            topic_items = list(self.node.topic_statuses.items())

        for name, status in topic_items:
            if row >= y + h - 1:
                break
            topic_short = status.topic[-20:] if len(status.topic) > 20 else status.topic
            rate_str = (
                f"{status.actual_hz:>6.1f}Hz" if status.actual_hz > 0 else "   ---  "
            )
            exp_str = (
                f"{status.expected_hz:>4.1f}Hz" if status.expected_hz > 0 else "  var"
            )

            line = f"{topic_short:<22} {rate_str} {exp_str}"
            self.safe_addstr(row, x + 2, line)

            # Status with color
            stat_attr = self.get_status_attr(status.status)
            self.safe_addstr(row, x + w - 8, f"[{status.status:^4}]", stat_attr)
            row += 1

    def draw_tf_panel(self, y: int, x: int, w: int, h: int):
        """Draw TF frames panel."""
        self.draw_box(y, x, h, w, "TF FRAMES")

        row = y + 1
        # Header
        header = f"{'Transform':<24} {'Status':>6} {'Desc':<10}"
        self.safe_addstr(row, x + 2, header, curses.A_BOLD)
        row += 1
        self.safe_addstr(row, x + 2, "-" * (w - 4))
        row += 1

        # Thread-safe snapshot of TF statuses
        with self.node._status_lock:
            tf_items = list(self.node.tf_statuses)

        for tf_status in tf_items:
            if row >= y + h - 1:
                break
            transform = f"{tf_status.parent} -> {tf_status.child}"
            transform = transform[:24]

            status_str = "OK" if tf_status.connected else "FAIL"
            stat_attr = self.get_status_attr(
                status_str if tf_status.connected else "CRIT"
            )

            line = f"{transform:<24}"
            self.safe_addstr(row, x + 2, line)
            self.safe_addstr(row, x + 27, f"[{status_str:^4}]", stat_attr)
            self.safe_addstr(row, x + 35, tf_status.description[: w - 37], curses.A_DIM)
            row += 1

    def draw_lifecycle_panel(self, y: int, x: int, w: int, h: int):
        """Draw lifecycle nodes panel."""
        self.draw_box(y, x, h, w, "LIFECYCLE NODES")

        row = y + 1
        # Header
        header = f"{'Node':<22} {'State':>12}"
        self.safe_addstr(row, x + 2, header, curses.A_BOLD)
        row += 1
        self.safe_addstr(row, x + 2, "-" * (w - 4))
        row += 1

        # Thread-safe snapshot of lifecycle statuses
        with self.node._status_lock:
            lifecycle_items = list(self.node.lifecycle_statuses.items())

        for name, status in lifecycle_items:
            if row >= y + h - 1:
                break
            node_short = name[:22]
            state_str = status.state[:12]

            self.safe_addstr(row, x + 2, f"{node_short:<22}")
            stat_attr = self.get_status_attr(status.state)
            self.safe_addstr(row, x + 24, f"[{state_str:^10}]", stat_attr)
            row += 1

    def draw_config_panel(self, y: int, x: int, w: int, h: int):
        """Draw config files panel."""
        self.draw_box(y, x, h, w, "CONFIG FILES")

        row = y + 1
        # Header
        header = f"{'File':<28} {'Status':>6}"
        self.safe_addstr(row, x + 2, header, curses.A_BOLD)
        row += 1
        self.safe_addstr(row, x + 2, "-" * (w - 4))
        row += 1

        # Config statuses are only set at startup, but use lock for consistency
        with self.node._status_lock:
            config_items = list(self.node.config_statuses)

        for cfg in config_items:
            if row >= y + h - 1:
                break
            # Show just filename
            filename = os.path.basename(cfg.path)[:28]
            status_str = "OK" if cfg.exists else "MISS"
            stat_attr = self.get_status_attr("OK" if cfg.exists else "CRIT")

            self.safe_addstr(row, x + 2, f"{filename:<28}")
            self.safe_addstr(row, x + 31, f"[{status_str:^4}]", stat_attr)
            row += 1

    def draw_help_dialog(self):
        """Draw a centered help dialog listing all keyboard shortcuts."""
        if self.stdscr is None:
            return
        max_y, max_x = self.stdscr.getmaxyx()

        shortcuts = [
            ("q", "Quit the application"),
            ("h", "Toggle this help dialog"),
            ("d", "Change ROS_DOMAIN_ID"),
            ("e", "Edit diagnostics.yaml config"),
            ("l", "LiDAR scan view (braille render)"),
            ("n", "Network view (r:refresh neighbors)"),
            ("+/=", "Increase refresh interval (slower)"),
            ("-/_", "Decrease refresh interval (faster)"),
        ]

        # Calculate dialog dimensions
        max_desc_len = max(len(desc) for _, desc in shortcuts)
        box_w = min(max_x - 4, max_desc_len + 16)
        box_h = (
            len(shortcuts) + 6
        )  # title + header + separator + items + blank + footer
        box_y = max(0, max_y // 2 - box_h // 2)
        box_x = max(0, max_x // 2 - box_w // 2)

        # Clear the area behind the dialog
        for i in range(box_h):
            self.safe_addstr(box_y + i, box_x, " " * box_w)

        # Draw dialog box
        self.draw_box(box_y, box_x, box_h, box_w, "HELP - Keyboard Shortcuts")

        row = box_y + 1
        header = f"  {'Key':<8} {'Action'}"
        self.safe_addstr(row, box_x + 2, header, curses.A_BOLD)
        row += 1
        self.safe_addstr(row, box_x + 2, "-" * (box_w - 4))
        row += 1

        for key_str, desc in shortcuts:
            self.safe_addstr(
                row,
                box_x + 4,
                key_str,
                curses.color_pair(self.COLOR_INFO) | curses.A_BOLD,
            )
            self.safe_addstr(row, box_x + 12, desc)
            row += 1

        row += 1
        self.safe_addstr(row, box_x + 2, "Press h or Esc to close", curses.A_DIM)

    @staticmethod
    def _get_network_interfaces() -> List[Dict[str, str]]:
        """Get all IPv4 network interfaces and their addresses.

        Uses /proc/net and socket to avoid external dependencies.
        """
        interfaces = []
        try:
            # Parse /proc/net/if_inet6 won't help for IPv4; use socket + ioctl
            # or simply parse ip command output
            result = subprocess.run(
                ["ip", "-4", "-o", "addr", "show"],
                capture_output=True,
                text=True,
                timeout=5,
            )
            if result.returncode == 0:
                for line in result.stdout.strip().splitlines():
                    # Format: "2: eth0    inet 192.168.1.10/24 brd 192.168.1.255 scope global eth0"
                    parts = line.split()
                    if len(parts) >= 4:
                        iface = parts[1].rstrip(":")
                        addr = parts[3]  # includes /prefix
                        scope = ""
                        if "scope" in parts:
                            scope_idx = parts.index("scope") + 1
                            if scope_idx < len(parts):
                                scope = parts[scope_idx]
                        state = "UP"
                        interfaces.append(
                            {
                                "iface": iface,
                                "addr": addr,
                                "scope": scope,
                                "state": state,
                            }
                        )

            # Get link state for each interface
            result2 = subprocess.run(
                ["ip", "-o", "link", "show"],
                capture_output=True,
                text=True,
                timeout=5,
            )
            if result2.returncode == 0:
                link_states = {}
                for line in result2.stdout.strip().splitlines():
                    parts = line.split()
                    if len(parts) >= 3:
                        iface = parts[1].rstrip(":")
                        # Extract state from <...> flags
                        for p in parts:
                            if p.startswith("<") and p.endswith(">"):
                                flags = p.strip("<>").split(",")
                                if "UP" in flags and "LOWER_UP" in flags:
                                    link_states[iface] = "UP"
                                elif "UP" in flags:
                                    link_states[iface] = "NO-CARRIER"
                                else:
                                    link_states[iface] = "DOWN"
                                break
                # Update states
                for entry in interfaces:
                    if entry["iface"] in link_states:
                        entry["state"] = link_states[entry["iface"]]

        except (subprocess.TimeoutExpired, FileNotFoundError):
            pass

        # Also get hostname
        try:
            hostname = socket.gethostname()
            interfaces.insert(
                0, {"iface": "hostname", "addr": hostname, "scope": "", "state": ""}
            )
        except Exception:
            pass

        return interfaces

    @staticmethod
    def _get_neighbors(iface: str) -> List[Dict[str, str]]:
        """Get ARP/neighbor table entries for a given interface."""
        neighbors = []
        try:
            result = subprocess.run(
                ["ip", "-4", "neigh", "show", "dev", iface],
                capture_output=True,
                text=True,
                timeout=5,
            )
            if result.returncode == 0:
                for line in result.stdout.strip().splitlines():
                    if not line.strip():
                        continue
                    # Format: "192.168.1.1 lladdr aa:bb:cc:dd:ee:ff REACHABLE"
                    parts = line.split()
                    if len(parts) >= 1:
                        ip_addr = parts[0]
                        mac = ""
                        state = ""
                        if "lladdr" in parts:
                            mac_idx = parts.index("lladdr") + 1
                            if mac_idx < len(parts):
                                mac = parts[mac_idx]
                        # State is typically the last word
                        if parts[-1] in (
                            "REACHABLE",
                            "STALE",
                            "DELAY",
                            "PROBE",
                            "FAILED",
                            "NOARP",
                            "PERMANENT",
                            "INCOMPLETE",
                        ):
                            state = parts[-1]

                        # Try reverse DNS (non-blocking with short timeout)
                        hostname = ""
                        try:
                            hostname = socket.gethostbyaddr(ip_addr)[0]
                        except (socket.herror, socket.gaierror, OSError):
                            pass

                        neighbors.append(
                            {
                                "ip": ip_addr,
                                "mac": mac,
                                "state": state,
                                "hostname": hostname,
                            }
                        )
        except (subprocess.TimeoutExpired, FileNotFoundError):
            pass

        # Sort by IP address
        neighbors.sort(
            key=lambda n: tuple(int(x) for x in n["ip"].split(".") if x.isdigit())
        )
        return neighbors

    def draw_network_dialog(self):
        """Draw full-screen network view with selectable interfaces and neighbor list."""
        if self.stdscr is None:
            return
        max_y, max_x = self.stdscr.getmaxyx()

        # Refresh interface list each frame (lightweight)
        self._net_ifaces = self._get_network_interfaces()

        # Strip hostname entry for display separately
        hostname = ""
        ifaces = self._net_ifaces
        if ifaces and ifaces[0]["iface"] == "hostname":
            hostname = ifaces[0]["addr"]
            ifaces = ifaces[1:]

        # Clamp selection
        if ifaces:
            self._net_sel = max(0, min(self._net_sel, len(ifaces) - 1))
        else:
            self._net_sel = 0

        # Refresh neighbors when selection changes
        if self._net_sel != self._net_last_sel:
            self._net_last_sel = self._net_sel
            self._net_neighbor_scroll = 0
            if ifaces:
                sel_iface = ifaces[self._net_sel]["iface"]
                self._net_neighbors = self._get_neighbors(sel_iface)
            else:
                self._net_neighbors = []

        # Layout: full-screen overlay
        # Title bar (row 0), interface panel (left), neighbor panel (right),
        # footer (last row)
        left_w = min(56, max_x // 2)
        right_w = max_x - left_w

        # Title bar
        title = " NETWORK INTERFACES"
        if hostname:
            title += f" - {hostname}"
        title += " | Up/Down:select  n/Esc:close "
        self.safe_addstr(0, 0, title.ljust(max_x), curses.A_REVERSE | curses.A_BOLD)

        # ── Left panel: Interfaces ──
        self.draw_box(1, 0, max_y - 2, left_w, "Interfaces")
        row = 2
        # Header
        header = f"  {'Interface':<12} {'Address':<18} {'State'}"
        self.safe_addstr(row, 2, header[: left_w - 4], curses.A_BOLD)
        row += 1

        for idx, entry in enumerate(ifaces):
            if row >= max_y - 2:
                break
            iface = entry["iface"][:12]
            addr = entry["addr"][:18]
            state = entry["state"]

            is_selected = idx == self._net_sel
            line_attr = curses.A_REVERSE if is_selected else 0

            # Highlight marker
            marker = ">" if is_selected else " "
            self.safe_addstr(row, 2, marker, line_attr | curses.A_BOLD)
            self.safe_addstr(row, 3, f" {iface:<12}", line_attr)
            self.safe_addstr(
                row,
                16,
                f"{addr:<18}",
                line_attr | curses.color_pair(self.COLOR_INFO),
            )

            # State with color (keep highlight background if selected)
            if state == "UP":
                s_attr = curses.color_pair(self.COLOR_OK) | curses.A_BOLD
            elif state == "DOWN":
                s_attr = curses.color_pair(self.COLOR_CRIT) | curses.A_BOLD
            else:
                s_attr = curses.color_pair(self.COLOR_WARN) | curses.A_BOLD
            if is_selected:
                s_attr |= curses.A_REVERSE
            self.safe_addstr(row, 35, state[: left_w - 37], s_attr)
            row += 1

        if not ifaces:
            self.safe_addstr(
                row,
                4,
                "No IPv4 interfaces found",
                curses.color_pair(self.COLOR_WARN),
            )

        # ── Right panel: Neighbors for selected interface ──
        if ifaces:
            sel_name = ifaces[self._net_sel]["iface"]
            panel_title = f"Neighbors on {sel_name}"
        else:
            panel_title = "Neighbors"

        self.draw_box(1, left_w, max_y - 2, right_w, panel_title)

        row = 2
        neighbor_area_h = max_y - 5  # rows available for neighbor entries
        # Header
        n_header = f"  {'IP Address':<16} {'MAC Address':<18} {'Host':<16} {'State'}"
        self.safe_addstr(row, left_w + 2, n_header[: right_w - 4], curses.A_BOLD)
        row += 1

        # Clamp scroll
        max_scroll = max(0, len(self._net_neighbors) - neighbor_area_h)
        self._net_neighbor_scroll = max(0, min(self._net_neighbor_scroll, max_scroll))

        visible_neighbors = self._net_neighbors[
            self._net_neighbor_scroll : self._net_neighbor_scroll + neighbor_area_h
        ]

        for entry in visible_neighbors:
            if row >= max_y - 2:
                break
            ip_addr = entry["ip"][:16]
            mac = entry["mac"][:18] if entry["mac"] else "incomplete"
            host = entry["hostname"][:16] if entry["hostname"] else ""
            state = entry["state"]

            self.safe_addstr(row, left_w + 4, f"{ip_addr:<16}")
            self.safe_addstr(
                row,
                left_w + 20,
                f"{mac:<18}",
                curses.color_pair(self.COLOR_INFO),
            )
            self.safe_addstr(row, left_w + 38, f"{host:<16}", curses.A_DIM)

            # State color
            if state in ("REACHABLE", "PERMANENT", "NOARP"):
                s_attr = curses.color_pair(self.COLOR_OK)
            elif state in ("STALE", "DELAY", "PROBE"):
                s_attr = curses.color_pair(self.COLOR_WARN)
            else:
                s_attr = curses.color_pair(self.COLOR_CRIT)
            self.safe_addstr(row, left_w + 54, state[: right_w - 56], s_attr)
            row += 1

        if not self._net_neighbors:
            self.safe_addstr(
                row,
                left_w + 4,
                "No neighbors found",
                curses.color_pair(self.COLOR_WARN),
            )

        # Scroll indicator
        if len(self._net_neighbors) > neighbor_area_h:
            scroll_info = (
                f" [{self._net_neighbor_scroll + 1}-"
                f"{min(self._net_neighbor_scroll + neighbor_area_h, len(self._net_neighbors))}"
                f"/{len(self._net_neighbors)}] PgUp/PgDn to scroll "
            )
            self.safe_addstr(
                max_y - 2, left_w + 2, scroll_info[: right_w - 4], curses.A_DIM
            )

        # Footer
        count = len(self._net_neighbors)
        footer = f" {count} neighbor(s) | Up/Down:select interface  PgUp/PgDn:scroll neighbors  n/Esc:close "
        self.safe_addstr(max_y - 1, 0, footer.ljust(max_x), curses.A_REVERSE)

    def _draw_yaml_line(self, row: int, col: int, text: str, max_w: int):
        """Draw a single line of YAML with syntax highlighting."""
        if not text:
            return
        text = text[:max_w]

        # Full-line comment
        stripped = text.lstrip()
        if stripped.startswith("#"):
            self.safe_addstr(row, col, text, curses.color_pair(self.COLOR_YAML_COMMENT))
            return

        # List item prefix (- )
        indent_end = len(text) - len(text.lstrip())
        prefix = text[:indent_end]
        rest = text[indent_end:]

        x = col
        # Draw leading whitespace
        if prefix:
            self.safe_addstr(row, x, prefix)
            x += len(prefix)

        # Handle list item dash
        if rest.startswith("- "):
            self.safe_addstr(row, x, "- ", curses.A_BOLD)
            x += 2
            rest = rest[2:]

        # Check for inline comment
        comment_part = ""
        in_quote = False
        comment_idx = -1
        for i, ch in enumerate(rest):
            if ch in ('"', "'") and (i == 0 or rest[i - 1] != "\\"):
                in_quote = not in_quote
            elif ch == "#" and not in_quote and i > 0 and rest[i - 1] == " ":
                comment_idx = i
                break

        if comment_idx >= 0:
            comment_part = rest[comment_idx:]
            rest = rest[:comment_idx]

        # Key: value pair
        colon_idx = rest.find(": ")
        if colon_idx == -1 and rest.endswith(":"):
            colon_idx = len(rest) - 1

        if colon_idx >= 0:
            key_part = rest[: colon_idx + 1]
            value_part = rest[colon_idx + 1 :]

            # Draw key
            self.safe_addstr(
                row, x, key_part, curses.color_pair(self.COLOR_YAML_KEY) | curses.A_BOLD
            )
            x += len(key_part)

            # Draw value
            if value_part:
                val_stripped = value_part.lstrip()
                val_leading = value_part[: len(value_part) - len(val_stripped)]
                self.safe_addstr(row, x, val_leading)
                x += len(val_leading)

                # Determine value type for coloring
                if val_stripped.startswith('"') or val_stripped.startswith("'"):
                    attr = curses.color_pair(self.COLOR_YAML_STRING)
                elif val_stripped in ("true", "false", "True", "False", "yes", "no"):
                    attr = curses.color_pair(self.COLOR_YAML_BOOL) | curses.A_BOLD
                elif val_stripped and (
                    val_stripped[0].isdigit()
                    or (val_stripped[0] in "-." and len(val_stripped) > 1)
                ):
                    attr = curses.color_pair(self.COLOR_YAML_NUMBER)
                else:
                    attr = curses.color_pair(self.COLOR_YAML_VALUE)
                self.safe_addstr(row, x, val_stripped, attr)
                x += len(val_stripped)
        else:
            # Plain value (e.g., list item value)
            val = rest.strip()
            leading = rest[: len(rest) - len(rest.lstrip())]
            if leading:
                self.safe_addstr(row, x, leading)
                x += len(leading)
            if val:
                if val.startswith('"') or val.startswith("'"):
                    attr = curses.color_pair(self.COLOR_YAML_STRING)
                elif val in ("true", "false", "True", "False", "yes", "no"):
                    attr = curses.color_pair(self.COLOR_YAML_BOOL) | curses.A_BOLD
                elif val and (
                    val[0].isdigit()
                    or (val[0] in "-." and len(val) > 1 and val[1:2].isdigit())
                ):
                    attr = curses.color_pair(self.COLOR_YAML_NUMBER)
                else:
                    attr = curses.color_pair(self.COLOR_YAML_VALUE)
                self.safe_addstr(row, x, val, attr)
                x += len(val)

        # Draw inline comment
        if comment_part:
            self.safe_addstr(
                row, x, comment_part, curses.color_pair(self.COLOR_YAML_COMMENT)
            )

    # Braille dot bit values: BRAILLE_DOT[row][col] for a 2-wide x 4-tall cell
    _BRAILLE_DOT = [
        [0x01, 0x08],  # row 0
        [0x02, 0x10],  # row 1
        [0x04, 0x20],  # row 2
        [0x40, 0x80],  # row 3
    ]

    def draw_lidar_view(self):
        """Draw full-screen LiDAR scan visualization using braille characters."""
        if self.stdscr is None:
            return
        max_y, max_x = self.stdscr.getmaxyx()

        # Reserve: row 0 for title, row max_y-1 for footer
        chart_h = max_y - 2  # character rows for the canvas
        chart_w = max_x  # character columns for the canvas
        if chart_h < 4 or chart_w < 10:
            return

        # Pixel resolution (braille gives 2x horizontal, 4x vertical)
        px_w = chart_w * 2
        px_h = chart_h * 4

        # Center in pixel space
        cx = px_w / 2.0
        cy = px_h / 2.0

        # Thread-safe snapshot of scan data
        with self.node._status_lock:
            scan = self.node.lidar_scan
            if scan.ranges is None or len(scan.ranges) == 0:
                has_data = False
                ranges = []
                angle_min = 0.0
                angle_inc = 0.0
                range_min = 0.0
                range_max = 0.0
                scan_ts = 0.0
                num_pts = 0
            else:
                has_data = True
                ranges = list(scan.ranges)
                angle_min = scan.angle_min
                angle_inc = scan.angle_increment
                range_min = scan.range_min
                range_max = scan.range_max
                scan_ts = scan.timestamp
                num_pts = len(ranges)

        # Title bar
        if has_data:
            age = time.time() - scan_ts
            title = (
                f" LIDAR SCAN | {num_pts} pts | "
                f"range {range_min:.1f}-{range_max:.1f}m | "
                f"age {age:.1f}s | l/Esc:close "
            )
        else:
            title = " LIDAR SCAN | Waiting for /scan data... | l/Esc:close "
        self.safe_addstr(0, 0, title.ljust(max_x), curses.A_REVERSE | curses.A_BOLD)

        if not has_data:
            self.safe_addstr(
                max_y // 2,
                max_x // 2 - 12,
                "No scan data received",
                curses.color_pair(self.COLOR_WARN) | curses.A_BOLD,
            )
            self.safe_addstr(max_y - 1, 0, " ".ljust(max_x), curses.A_REVERSE)
            return

        # Find actual max range for auto-scaling
        actual_max = 0.0
        valid_count = 0
        for r in ranges:
            if range_min <= r <= range_max and math.isfinite(r):
                if r > actual_max:
                    actual_max = r
                valid_count += 1

        if actual_max <= 0:
            actual_max = range_max

        # View radius in pixels, with margin
        view_radius = min(px_w, px_h) / 2.0 - 4
        scale = view_radius / actual_max if actual_max > 0 else 1.0

        # Initialize braille cell grid and a separate grid for range rings
        cells = [[0] * chart_w for _ in range(chart_h)]
        grid_cells = [[0] * chart_w for _ in range(chart_h)]

        # Draw range rings at nice intervals
        if actual_max > 0:
            # Pick ring spacing: 0.5, 1, 2, 5, 10, 20, 50 meters
            ring_options = [0.5, 1.0, 2.0, 5.0, 10.0, 20.0, 50.0]
            ring_step = ring_options[0]
            for opt in ring_options:
                if actual_max / opt <= 6:
                    ring_step = opt
                    break

            ring_r = ring_step
            while ring_r < actual_max:
                # Draw circle: sample points around 360 degrees
                r_px = ring_r * scale
                for deg in range(360):
                    theta = math.radians(deg)
                    px = int(cx - r_px * math.sin(theta))
                    py = int(cy - r_px * math.cos(theta))
                    if 0 <= px < px_w and 0 <= py < px_h:
                        cell_x = px // 2
                        cell_y = py // 4
                        dot_x = px % 2
                        dot_y = py % 4
                        if 0 <= cell_x < chart_w and 0 <= cell_y < chart_h:
                            grid_cells[cell_y][cell_x] |= self._BRAILLE_DOT[dot_y][
                                dot_x
                            ]
                ring_r += ring_step

            # Draw cross axes through center
            for i in range(px_h):
                # Vertical line (forward axis)
                px = int(cx)
                if 0 <= px < px_w and 0 <= i < px_h:
                    cell_x = px // 2
                    cell_y = i // 4
                    dot_x = px % 2
                    dot_y = i % 4
                    if 0 <= cell_x < chart_w and 0 <= cell_y < chart_h:
                        grid_cells[cell_y][cell_x] |= self._BRAILLE_DOT[dot_y][dot_x]
            for i in range(px_w):
                # Horizontal line (left-right axis)
                py = int(cy)
                if 0 <= i < px_w and 0 <= py < px_h:
                    cell_x = i // 2
                    cell_y = py // 4
                    dot_x = i % 2
                    dot_y = py % 4
                    if 0 <= cell_x < chart_w and 0 <= cell_y < chart_h:
                        grid_cells[cell_y][cell_x] |= self._BRAILLE_DOT[dot_y][dot_x]

        # Plot scan points
        angle = angle_min
        for r in ranges:
            if range_min <= r <= range_max and math.isfinite(r):
                # ROS: x=r*cos(a) forward, y=r*sin(a) left
                # Screen: x right, y down; forward=up, left=left
                px = int(cx - r * math.sin(angle) * scale)
                py = int(cy - r * math.cos(angle) * scale)
                if 0 <= px < px_w and 0 <= py < px_h:
                    cell_x = px // 2
                    cell_y = py // 4
                    dot_x = px % 2
                    dot_y = py % 4
                    if 0 <= cell_x < chart_w and 0 <= cell_y < chart_h:
                        cells[cell_y][cell_x] |= self._BRAILLE_DOT[dot_y][dot_x]
            angle += angle_inc

        # Render braille characters to screen
        scan_attr = curses.color_pair(self.COLOR_LIDAR_SCAN) | curses.A_BOLD
        grid_attr = curses.color_pair(self.COLOR_LIDAR_GRID)
        for row in range(chart_h):
            line_chars = []
            line_attrs = []
            for col in range(chart_w):
                scan_val = cells[row][col]
                grid_val = grid_cells[row][col]
                if scan_val:
                    # Scan points take priority — merge with grid for completeness
                    line_chars.append(chr(0x2800 + (scan_val | grid_val)))
                    line_attrs.append(scan_attr)
                elif grid_val:
                    line_chars.append(chr(0x2800 + grid_val))
                    line_attrs.append(grid_attr)
                else:
                    line_chars.append(" ")
                    line_attrs.append(0)

            # Draw row character by character (for per-cell color)
            screen_row = row + 1  # offset for title bar
            for col, (ch, attr) in enumerate(zip(line_chars, line_attrs)):
                if ch != " ":
                    self.safe_addstr(screen_row, col, ch, attr)

        # Robot marker at center
        center_row = int(cy / 4) + 1
        center_col = int(cx / 2)
        self.safe_addstr(
            center_row,
            center_col,
            "+",
            curses.color_pair(self.COLOR_CRIT) | curses.A_BOLD,
        )

        # Range labels on rings
        if actual_max > 0 and ring_step > 0:
            ring_r = ring_step
            while ring_r < actual_max:
                # Place label above center on the vertical axis
                label_py = int(cy - ring_r * scale)
                label_row = label_py // 4 + 1
                label_col = int(cx / 2) + 1
                if 1 <= label_row < max_y - 1:
                    label = (
                        f"{ring_r:.0f}m" if ring_r == int(ring_r) else f"{ring_r:.1f}m"
                    )
                    self.safe_addstr(
                        label_row,
                        label_col,
                        label,
                        curses.color_pair(self.COLOR_INFO) | curses.A_DIM,
                    )
                ring_r += ring_step

        # Footer
        fov_deg = math.degrees(angle_min + angle_inc * num_pts) - math.degrees(
            angle_min
        )
        footer = (
            f" {valid_count}/{num_pts} valid pts | "
            f"FOV {abs(fov_deg):.0f}deg | "
            f"max {actual_max:.1f}m | "
            f"ring {ring_step:.1f}m | "
            f"l/Esc:close "
        )
        self.safe_addstr(max_y - 1, 0, footer.ljust(max_x), curses.A_REVERSE)

    def _get_config_path(self) -> str:
        """Return path to the installed diagnostics.yaml."""
        try:
            pkg_share = get_package_share_directory("autonomy_diagnostics")
            return os.path.join(pkg_share, "config", "diagnostics.yaml")
        except Exception:
            return ""

    def edit_config(self):
        """Full-screen curses text editor for diagnostics.yaml."""
        if self.stdscr is None:
            return
        config_path = self._get_config_path()
        if not config_path or not os.path.isfile(config_path):
            return

        # Load file content
        with open(config_path, "r") as f:
            lines = f.read().splitlines()
        if not lines:
            lines = [""]

        cursor_y = 0  # Line index
        cursor_x = 0  # Column index
        scroll_y = 0  # First visible line
        scroll_x = 0  # First visible column
        modified = False
        status_msg = ""

        # Switch to blocking input
        self.stdscr.nodelay(False)
        curses.curs_set(1)

        # Disable XON/XOFF flow control so Ctrl+S reaches the application
        # instead of freezing the terminal
        fd = sys.stdin.fileno()
        old_term = termios.tcgetattr(fd)
        new_term = termios.tcgetattr(fd)
        new_term[0] &= ~termios.IXON  # Disable IXON in input flags
        termios.tcsetattr(fd, termios.TCSADRAIN, new_term)

        try:
            while True:
                self.stdscr.erase()
                max_y, max_x = self.stdscr.getmaxyx()

                # Reserve rows: 1 for title, 1 for status bar
                edit_h = max_y - 2
                gutter_w = 5  # Width for line numbers
                text_w = max_x - gutter_w - 1

                # Title bar
                mod_indicator = " [modified]" if modified else ""
                title = f" EDIT: {os.path.basename(config_path)}{mod_indicator} "
                title += "| Ctrl-S/F2:save  Esc:close "
                self.safe_addstr(
                    0, 0, title.ljust(max_x), curses.A_REVERSE | curses.A_BOLD
                )

                # Clamp scroll so cursor is always visible
                if cursor_y < scroll_y:
                    scroll_y = cursor_y
                elif cursor_y >= scroll_y + edit_h:
                    scroll_y = cursor_y - edit_h + 1
                if cursor_x < scroll_x:
                    scroll_x = cursor_x
                elif cursor_x >= scroll_x + text_w:
                    scroll_x = cursor_x - text_w + 1

                # Draw lines with YAML syntax highlighting
                for i in range(edit_h):
                    line_idx = scroll_y + i
                    row = i + 1  # +1 for title bar
                    if line_idx < len(lines):
                        # Line number gutter
                        ln_str = f"{line_idx + 1:>4} "
                        self.safe_addstr(row, 0, ln_str, curses.A_DIM)
                        # Line content (scrolled horizontally) with highlighting
                        visible = lines[line_idx][scroll_x : scroll_x + text_w]
                        self._draw_yaml_line(row, gutter_w, visible, text_w)
                    else:
                        self.safe_addstr(row, 0, "   ~ ", curses.A_DIM)

                # Status bar
                pos_info = (
                    f" Ln {cursor_y + 1}, Col {cursor_x + 1} | {len(lines)} lines "
                )
                if status_msg:
                    bar = f" {status_msg} | {pos_info}"
                else:
                    bar = pos_info
                self.safe_addstr(max_y - 1, 0, bar.ljust(max_x), curses.A_REVERSE)

                # Position cursor
                screen_cy = cursor_y - scroll_y + 1
                screen_cx = cursor_x - scroll_x + gutter_w
                try:
                    self.stdscr.move(screen_cy, screen_cx)
                except curses.error:
                    pass

                self.stdscr.refresh()
                key = self.stdscr.getch()
                status_msg = ""

                # Navigation
                if key == curses.KEY_UP:
                    if cursor_y > 0:
                        cursor_y -= 1
                        cursor_x = min(cursor_x, len(lines[cursor_y]))
                elif key == curses.KEY_DOWN:
                    if cursor_y < len(lines) - 1:
                        cursor_y += 1
                        cursor_x = min(cursor_x, len(lines[cursor_y]))
                elif key == curses.KEY_LEFT:
                    if cursor_x > 0:
                        cursor_x -= 1
                    elif cursor_y > 0:
                        cursor_y -= 1
                        cursor_x = len(lines[cursor_y])
                elif key == curses.KEY_RIGHT:
                    if cursor_x < len(lines[cursor_y]):
                        cursor_x += 1
                    elif cursor_y < len(lines) - 1:
                        cursor_y += 1
                        cursor_x = 0
                elif key == curses.KEY_HOME:
                    cursor_x = 0
                elif key == curses.KEY_END:
                    cursor_x = len(lines[cursor_y])
                elif key == curses.KEY_PPAGE:  # Page Up
                    cursor_y = max(0, cursor_y - edit_h)
                    cursor_x = min(cursor_x, len(lines[cursor_y]))
                elif key == curses.KEY_NPAGE:  # Page Down
                    cursor_y = min(len(lines) - 1, cursor_y + edit_h)
                    cursor_x = min(cursor_x, len(lines[cursor_y]))

                # Save: Ctrl+S or F2
                elif key in (19, curses.KEY_F2):
                    try:
                        content = "\n".join(lines) + "\n"
                        with open(config_path, "w") as f:
                            f.write(content)
                        modified = False
                        status_msg = "Saved! Restart (q then relaunch) to apply."
                    except PermissionError:
                        status_msg = (
                            "ERROR: Permission denied (installed config is read-only)"
                        )
                    except Exception as e:
                        status_msg = f"ERROR: {e}"

                # Cancel: Escape
                elif key == 27:
                    if modified:
                        # Confirm discard - draw prompt
                        self.safe_addstr(
                            max_y - 1,
                            0,
                            " Unsaved changes! Press Esc again to discard, any other key to resume ".ljust(
                                max_x
                            ),
                            curses.color_pair(self.COLOR_WARN) | curses.A_REVERSE,
                        )
                        self.stdscr.refresh()
                        confirm = self.stdscr.getch()
                        if confirm == 27:
                            break
                    else:
                        break

                # Backspace
                elif key in (curses.KEY_BACKSPACE, 127, 8):
                    if cursor_x > 0:
                        line = lines[cursor_y]
                        lines[cursor_y] = line[: cursor_x - 1] + line[cursor_x:]
                        cursor_x -= 1
                        modified = True
                    elif cursor_y > 0:
                        # Join with previous line
                        cursor_x = len(lines[cursor_y - 1])
                        lines[cursor_y - 1] += lines[cursor_y]
                        del lines[cursor_y]
                        cursor_y -= 1
                        modified = True

                # Delete
                elif key == curses.KEY_DC:
                    line = lines[cursor_y]
                    if cursor_x < len(line):
                        lines[cursor_y] = line[:cursor_x] + line[cursor_x + 1 :]
                        modified = True
                    elif cursor_y < len(lines) - 1:
                        lines[cursor_y] += lines[cursor_y + 1]
                        del lines[cursor_y + 1]
                        modified = True

                # Enter
                elif key in (curses.KEY_ENTER, 10, 13):
                    line = lines[cursor_y]
                    # Auto-indent: carry over leading whitespace
                    indent = ""
                    for ch in line:
                        if ch in (" ", "\t"):
                            indent += ch
                        else:
                            break
                    lines[cursor_y] = line[:cursor_x]
                    lines.insert(cursor_y + 1, indent + line[cursor_x:])
                    cursor_y += 1
                    cursor_x = len(indent)
                    modified = True

                # Tab -> 2 spaces (YAML convention)
                elif key == 9:
                    line = lines[cursor_y]
                    lines[cursor_y] = line[:cursor_x] + "  " + line[cursor_x:]
                    cursor_x += 2
                    modified = True

                # Regular character
                elif 32 <= key <= 126:
                    line = lines[cursor_y]
                    lines[cursor_y] = line[:cursor_x] + chr(key) + line[cursor_x:]
                    cursor_x += 1
                    modified = True

        finally:
            # Restore terminal flow control settings
            termios.tcsetattr(fd, termios.TCSADRAIN, old_term)
            curses.curs_set(0)
            self.stdscr.nodelay(True)
            self.stdscr.timeout(self.refresh_ms)

    def prompt_domain_id(self) -> Optional[int]:
        """Show a popup prompt for entering a new ROS_DOMAIN_ID. Returns the ID or None."""
        if self.stdscr is None:
            return None
        max_y, max_x = self.stdscr.getmaxyx()

        # Popup dimensions and position
        box_w = 40
        box_h = 5
        box_y = max_y // 2 - box_h // 2
        box_x = max_x // 2 - box_w // 2

        current_id = os.environ.get("ROS_DOMAIN_ID", "0")
        input_buf = ""

        while True:
            # Draw popup
            self.draw_box(box_y, box_x, box_h, box_w, "Set ROS_DOMAIN_ID")
            self.safe_addstr(
                box_y + 1, box_x + 2, f"Current: {current_id}", curses.A_DIM
            )
            prompt_line = f"New ID (0-232): {input_buf}_"
            self.safe_addstr(box_y + 2, box_x + 2, " " * (box_w - 4))
            self.safe_addstr(box_y + 2, box_x + 2, prompt_line)
            self.safe_addstr(
                box_y + 3, box_x + 2, "Enter=confirm  Esc=cancel", curses.A_DIM
            )
            self.stdscr.refresh()

            # Block for input (override nodelay temporarily)
            self.stdscr.nodelay(False)
            try:
                key = self.stdscr.getch()
            finally:
                self.stdscr.nodelay(True)
                self.stdscr.timeout(self.refresh_ms)

            if key == 27:  # Escape
                return None
            elif key in (curses.KEY_ENTER, 10, 13):
                if input_buf == "":
                    return None
                value = int(input_buf)
                if 0 <= value <= 232:
                    return value
                # Invalid range — flash and let them retry
                input_buf = ""
            elif key in (curses.KEY_BACKSPACE, 127, 8):
                input_buf = input_buf[:-1]
            elif ord("0") <= key <= ord("9") and len(input_buf) < 3:
                input_buf += chr(key)

    def run_curses(self, stdscr):
        """Main curses loop."""
        self.stdscr = stdscr
        curses.curs_set(0)  # Hide cursor
        stdscr.nodelay(True)  # Non-blocking input
        stdscr.timeout(self.refresh_ms)

        # Initialize color pairs
        curses.start_color()
        curses.use_default_colors()
        curses.init_pair(self.COLOR_OK, curses.COLOR_GREEN, -1)
        curses.init_pair(self.COLOR_WARN, curses.COLOR_YELLOW, -1)
        curses.init_pair(self.COLOR_CRIT, curses.COLOR_RED, -1)
        curses.init_pair(self.COLOR_INFO, curses.COLOR_CYAN, -1)
        # YAML editor syntax colors
        curses.init_pair(self.COLOR_YAML_KEY, curses.COLOR_CYAN, -1)
        curses.init_pair(self.COLOR_YAML_VALUE, curses.COLOR_WHITE, -1)
        curses.init_pair(self.COLOR_YAML_COMMENT, curses.COLOR_GREEN, -1)
        curses.init_pair(self.COLOR_YAML_STRING, curses.COLOR_YELLOW, -1)
        curses.init_pair(self.COLOR_YAML_BOOL, curses.COLOR_MAGENTA, -1)
        curses.init_pair(self.COLOR_YAML_NUMBER, curses.COLOR_RED, -1)
        # LiDAR view colors
        curses.init_pair(self.COLOR_LIDAR_SCAN, curses.COLOR_GREEN, -1)
        curses.init_pair(self.COLOR_LIDAR_GRID, curses.COLOR_BLUE, -1)

        while self.running:
            try:
                # Use erase() instead of clear() to avoid screen flicker
                # clear() forces a full terminal redraw, erase() just overwrites
                stdscr.erase()
                max_y, max_x = stdscr.getmaxyx()

                if self.show_lidar:
                    # Full-screen LiDAR scan view
                    self.draw_lidar_view()
                elif self.show_network:
                    # Full-screen network view
                    self.draw_network_dialog()
                else:
                    # Normal dashboard view
                    # Title bar
                    title = " AUTONOMY DIAGNOSTICS - q:quit  d:domain  e:edit  n:network  l:lidar  +/-:refresh  h:help "
                    self.safe_addstr(
                        0, 0, title.center(max_x), curses.A_REVERSE | curses.A_BOLD
                    )

                    # Summary bar
                    self.draw_summary_bar(1, max_x)

                    # Calculate panel dimensions dynamically based on content
                    half_x = max_x // 2
                    available_height = (
                        max_y - 4
                    )  # Title bar, summary bar, status bar, margin

                    # Get content counts for sizing (thread-safe)
                    with self.node._status_lock:
                        num_topics = len(self.node.topic_statuses)
                        num_tf = len(self.node.tf_statuses)
                        num_lifecycle = len(self.node.lifecycle_statuses)
                        num_config = len(self.node.config_statuses)

                    # Calculate heights: header(1) + separator(1) + items + border(2)
                    topics_needed = num_topics + 4
                    tf_needed = num_tf + 4
                    lifecycle_needed = num_lifecycle + 4
                    config_needed = num_config + 4

                    # Top row needs max of topics/tf, bottom row needs max of lifecycle/config
                    top_row_min = max(topics_needed, tf_needed)
                    bottom_row_min = max(lifecycle_needed, config_needed)

                    # Distribute available height proportionally, with minimums
                    total_needed = top_row_min + bottom_row_min
                    if total_needed <= available_height:
                        panel_h_top = top_row_min
                        panel_h_bot = available_height - panel_h_top
                    else:
                        # Not enough space - split proportionally
                        ratio = top_row_min / total_needed
                        panel_h_top = max(6, int(available_height * ratio))
                        panel_h_bot = max(6, available_height - panel_h_top)

                    # Draw panels - 2x2 grid
                    # Top left: Topics
                    self.draw_topics_panel(2, 0, half_x, panel_h_top)

                    # Top right: TF Frames
                    self.draw_tf_panel(2, half_x, max_x - half_x, panel_h_top)

                    # Bottom left: Lifecycle Nodes
                    self.draw_lifecycle_panel(2 + panel_h_top, 0, half_x, panel_h_bot)

                    # Bottom right: Config Files
                    self.draw_config_panel(
                        2 + panel_h_top, half_x, max_x - half_x, panel_h_bot
                    )

                    # Help dialog overlay (drawn on top of panels)
                    if self.show_help:
                        self.draw_help_dialog()

                    # Status bar
                    status_time = time.strftime("%H:%M:%S")
                    domain_id = os.environ.get("ROS_DOMAIN_ID", "0")
                    status = f" Time: {status_time} | ROS_DOMAIN_ID: {domain_id} | Refresh: {self.refresh_ms}ms"

                    # Joystick status in status bar
                    with self.node._status_lock:
                        joy = self.node.joystick_status
                        joy_device = joy.device_connected
                        joy_path = (
                            os.path.basename(joy.device_path) if joy.device_path else ""
                        )
                        joy_axes = joy.num_axes
                        joy_buttons = joy.num_buttons
                        joy_active = joy.last_msg_time > 0 and (
                            time.time() - joy.last_msg_time
                            < self.node.stale_data_timeout
                        )

                    if joy_device and joy_active:
                        joy_str = f"{joy_path} {joy_axes}ax/{joy_buttons}btn"
                        joy_status = "OK"
                    elif joy_device:
                        joy_str = f"{joy_path} (idle)"
                        joy_status = "WARN"
                    else:
                        joy_str = "no device"
                        joy_status = "NONE"

                    status += f" | Joy: {joy_str} "
                    self.safe_addstr(
                        max_y - 1, 0, status.ljust(max_x), curses.A_REVERSE
                    )

                    # Color the joy status portion
                    joy_label_pos = status.find("Joy: ") + 5
                    joy_attr = self.get_status_attr(joy_status) | curses.A_REVERSE
                    self.safe_addstr(max_y - 1, joy_label_pos, joy_str, joy_attr)

                stdscr.refresh()

                # Handle keyboard input
                key = stdscr.getch()
                if self.show_help:
                    # While help is open, only h/H/Esc dismiss it
                    if key in (ord("h"), ord("H"), 27):
                        self.show_help = False
                    elif key == ord("q") or key == ord("Q"):
                        self.running = False
                    continue
                if self.show_network:
                    if key in (ord("n"), ord("N"), 27):
                        self.show_network = False
                        self._net_last_sel = -1  # Reset so neighbors refresh on reopen
                    elif key == ord("q") or key == ord("Q"):
                        self.running = False
                    elif key == curses.KEY_UP:
                        self._net_sel = max(0, self._net_sel - 1)
                    elif key == curses.KEY_DOWN:
                        iface_count = len(self._net_ifaces)
                        # Subtract 1 for hostname entry if present
                        if (
                            self._net_ifaces
                            and self._net_ifaces[0]["iface"] == "hostname"
                        ):
                            iface_count -= 1
                        self._net_sel = min(iface_count - 1, self._net_sel + 1)
                    elif key == curses.KEY_PPAGE:
                        self._net_neighbor_scroll = max(
                            0, self._net_neighbor_scroll - (max_y - 6)
                        )
                    elif key == curses.KEY_NPAGE:
                        self._net_neighbor_scroll += max_y - 6
                    elif key == ord("r") or key == ord("R"):
                        # Force refresh neighbors
                        self._net_last_sel = -1
                    continue
                if self.show_lidar:
                    if key in (ord("l"), ord("L"), 27):
                        self.show_lidar = False
                    elif key == ord("q") or key == ord("Q"):
                        self.running = False
                    continue
                if key == ord("q") or key == ord("Q"):
                    self.running = False
                elif key == ord("h") or key == ord("H"):
                    self.show_help = True
                elif key == ord("d") or key == ord("D"):
                    new_id = self.prompt_domain_id()
                    if new_id is not None:
                        self.new_domain_id = new_id
                        self.running = False  # Signal restart
                elif key == ord("e") or key == ord("E"):
                    self.edit_config()
                elif key == ord("n") or key == ord("N"):
                    self.show_network = True
                elif key == ord("l") or key == ord("L"):
                    self.show_lidar = True
                elif key == ord("+") or key == ord("="):
                    # Increase refresh rate (slower updates)
                    self.refresh_ms = min(
                        self.refresh_ms + self.REFRESH_STEP_MS, self.REFRESH_MAX_MS
                    )
                    stdscr.timeout(self.refresh_ms)
                elif key == ord("-") or key == ord("_"):
                    # Decrease refresh rate (faster updates)
                    self.refresh_ms = max(
                        self.refresh_ms - self.REFRESH_STEP_MS, self.REFRESH_MIN_MS
                    )
                    stdscr.timeout(self.refresh_ms)

            except curses.error:
                pass
            except KeyboardInterrupt:
                self.running = False

    def run_simple(self):
        """Simple text output mode for non-TTY environments."""
        print("AUTONOMY DIAGNOSTICS - Simple Mode (no TTY detected)")
        print("=" * 70)
        print("Press Ctrl+C to exit\n")

        while self.running:
            try:
                # Clear screen (ANSI escape)
                print("\033[2J\033[H", end="")
                print("=" * 70)
                print(" AUTONOMY DIAGNOSTICS - Simple Mode")
                print("=" * 70)

                # Topics
                print("\n[TOPICS]")
                for name, status in self.node.topic_statuses.items():
                    rate_str = (
                        f"{status.actual_hz:.1f}Hz" if status.actual_hz > 0 else "---"
                    )
                    print(f"  {status.topic:<25} {rate_str:>8}  [{status.status}]")

                # TF Frames
                print("\n[TF FRAMES]")
                for tf_status in self.node.tf_statuses:
                    status_str = "OK" if tf_status.connected else "FAIL"
                    print(
                        f"  {tf_status.parent} -> {tf_status.child:<15} [{status_str}] {tf_status.description}"
                    )

                # Lifecycle Nodes
                print("\n[LIFECYCLE NODES]")
                for name, status in self.node.lifecycle_statuses.items():
                    print(f"  {name:<22} [{status.state}]")

                # Config Files
                print("\n[CONFIG FILES]")
                for cfg in self.node.config_statuses:
                    status_str = "OK" if cfg.exists else "MISSING"
                    print(f"  {cfg.description:<20} [{status_str}]")

                # Joystick
                print("\n[JOYSTICK]")
                joy = self.node.joystick_status
                if joy.device_connected and joy.last_msg_time > 0:
                    print(
                        f"  Device: {joy.device_path}  "
                        f"Axes: {joy.num_axes}  Buttons: {joy.num_buttons}  [OK]"
                    )
                elif joy.device_connected:
                    print(f"  Device: {joy.device_path}  [IDLE - no messages]")
                else:
                    print("  No joystick device detected")

                print("\n" + "=" * 70)
                domain_id = os.environ.get("ROS_DOMAIN_ID", "0")
                print(
                    f"Updated: {time.strftime('%H:%M:%S')} | ROS_DOMAIN_ID: {domain_id}"
                )

                time.sleep(SIMPLE_MODE_REFRESH_INTERVAL)

            except KeyboardInterrupt:
                self.running = False
                break

    def run(self):
        """Run the TUI - uses curses if TTY available, otherwise simple text."""
        if os.isatty(sys.stdout.fileno()):
            try:
                curses.wrapper(self.run_curses)
            except curses.error as e:
                print(f"Curses error: {e}, falling back to simple mode")
                self.run_simple()
        else:
            self.run_simple()

    def stop(self):
        """Stop the TUI."""
        self.running = False


# =============================================================================
# Main
# =============================================================================


def parse_args() -> argparse.Namespace:
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(description="Autonomy Diagnostics TUI")
    parser.add_argument(
        "-d",
        "--domain-id",
        type=int,
        default=None,
        metavar="ID",
        help="Set the ROS_DOMAIN_ID (0-232). Overrides the environment variable.",
    )
    # Parse only known args so ROS args (e.g. --ros-args) pass through
    known, _ = parser.parse_known_args()
    return known


def main(args=None):
    """Main entry point."""
    cli_args = parse_args()

    if cli_args.domain_id is not None:
        if not 0 <= cli_args.domain_id <= 232:
            print(f"Error: domain-id must be 0-232, got {cli_args.domain_id}")
            sys.exit(1)
        os.environ["ROS_DOMAIN_ID"] = str(cli_args.domain_id)

    while True:
        rclpy.init(args=args)

        node = AutonomyDiagnosticsNode()
        tui = AutonomyTUI(node)

        # Handle SIGINT and SIGTERM for clean shutdown
        def signal_handler(sig, frame):
            tui.stop()

        signal.signal(signal.SIGINT, signal_handler)
        signal.signal(signal.SIGTERM, signal_handler)

        # Run ROS spinner in background
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()

        try:
            tui.run()
        finally:
            tui.stop()
            executor.shutdown()
            node.destroy_node()
            rclpy.shutdown()

        # Check if user requested a domain ID change (restart) or just quit
        if tui.new_domain_id is not None:
            os.environ["ROS_DOMAIN_ID"] = str(tui.new_domain_id)
            continue
        break


if __name__ == "__main__":
    main()
