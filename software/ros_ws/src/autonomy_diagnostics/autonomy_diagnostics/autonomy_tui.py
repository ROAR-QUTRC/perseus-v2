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

import curses
import logging
import os
import signal
import subprocess
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Any, Dict, List

import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import Imu, JointState, LaserScan
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

    @property
    def status(self) -> str:
        """Determine status based on rate and staleness."""
        now = time.time()
        if self.msg_count == 0:
            return "NONE"
        if now - self.last_msg_time > STALE_DATA_TIMEOUT:
            return "STALE"
        if self.expected_hz <= 0:
            return "OK"  # Variable rate topics
        ratio = self.actual_hz / self.expected_hz
        if ratio < RATE_CRITICAL_THRESHOLD:
            return "CRIT"
        if ratio < RATE_WARNING_THRESHOLD:
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

        # Check config files once
        self._check_config_files()

        # Create timers for periodic checks
        self.tf_timer = self.create_timer(
            TF_CHECK_INTERVAL, self._check_tf_frames, callback_group=self.cb_group
        )
        self.lifecycle_timer = self.create_timer(
            LIFECYCLE_CHECK_INTERVAL,
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
            )
            self.rate_calculators[name] = RateCalculator()

            # Create subscriber based on message type
            if msg_type == "LaserScan":
                self.create_subscription(
                    LaserScan,
                    topic,
                    lambda msg, n=name: self._topic_callback(n),
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

                        if age > STALE_DATA_TIMEOUT:
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

    # Color pair indices
    COLOR_OK = 1
    COLOR_WARN = 2
    COLOR_CRIT = 3
    COLOR_INFO = 4

    # Refresh rate bounds (ms)
    REFRESH_MIN_MS = 100
    REFRESH_MAX_MS = 2000
    REFRESH_STEP_MS = 100

    def __init__(self, node: AutonomyDiagnosticsNode):
        self.node = node
        self.running = True
        self.stdscr = None
        self.refresh_ms = CURSES_REFRESH_MS  # Mutable refresh rate

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

        while self.running:
            try:
                # Use erase() instead of clear() to avoid screen flicker
                # clear() forces a full terminal redraw, erase() just overwrites
                stdscr.erase()
                max_y, max_x = stdscr.getmaxyx()

                # Title bar
                title = " AUTONOMY DIAGNOSTICS - q:quit  +/-:refresh rate "
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

                # Status bar
                status_time = time.strftime("%H:%M:%S")
                domain_id = os.environ.get("ROS_DOMAIN_ID", "0")
                status = f" Time: {status_time} | ROS_DOMAIN_ID: {domain_id} | Refresh: {self.refresh_ms}ms "
                self.safe_addstr(max_y - 1, 0, status.ljust(max_x), curses.A_REVERSE)

                stdscr.refresh()

                # Handle keyboard input
                key = stdscr.getch()
                if key == ord("q") or key == ord("Q"):
                    self.running = False
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


def main(args=None):
    """Main entry point."""
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


if __name__ == "__main__":
    main()
