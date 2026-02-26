"""Main orchestrator ROS2 node for the mapping autotune system.

Runs a state machine that configures parameters, drives the robot through
maneuvers, captures maps, and scores them to find optimal SLAM/EKF settings.
"""

import json
import os
import signal
import subprocess
import threading
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import Imu, LaserScan

from mapping_autotune.db_manager import DbManager
from mapping_autotune.maneuver_executor import ManeuverExecutor
from mapping_autotune.map_analyzer import MapAnalyzer
from mapping_autotune.param_manager import ParamManager


class State:
    """State machine states for the autotune orchestrator."""

    WAITING_FOR_TUI = "waiting_for_tui"
    PREFLIGHT = "preflight"
    IDLE = "idle"
    CONFIGURING = "configuring"
    RESETTING_SLAM = "resetting_slam"
    SETTLING = "settling"
    RUNNING_MANEUVER = "running_maneuver"
    CAPTURING_MAP = "capturing_map"
    ANALYZING = "analyzing"
    COMPLETE = "complete"
    ERROR = "error"


class AutotuneNode(Node):
    """Orchestrator node that drives the mapping autotune state machine."""

    def __init__(self):
        super().__init__("autotune_node")

        # ── Declare ROS parameters ────────────────────────────────────
        self.declare_parameter("db_path", "~/.local/share/mapping_autotune/autotune.db")
        self.declare_parameter("max_runs", 10)
        self.declare_parameter("session_name", "")
        self.declare_parameter("session_description", "")
        self.declare_parameter("settling_time", 3.0)
        self.declare_parameter("slam_config_path", "")
        self.declare_parameter("ekf_config_path", "")
        self.declare_parameter("maneuver_pattern", "box_return")
        self.declare_parameter("linear_speed", 0.2)
        self.declare_parameter("rotation_speed", 0.5)
        self.declare_parameter("enabled_phases", "")
        self.declare_parameter("interactive", False)
        self.declare_parameter("skip_preflight", False)

        # ── Read parameter values ─────────────────────────────────────
        self._db_path = os.path.expanduser(self.get_parameter("db_path").value)
        self._max_runs = self.get_parameter("max_runs").value
        self._session_name = self.get_parameter("session_name").value
        self._session_description = self.get_parameter("session_description").value
        self._settling_time = self.get_parameter("settling_time").value
        self._slam_config_path = self.get_parameter("slam_config_path").value
        self._ekf_config_path = self.get_parameter("ekf_config_path").value
        self._maneuver_pattern = self.get_parameter("maneuver_pattern").value
        self._linear_speed = self.get_parameter("linear_speed").value
        self._rotation_speed = self.get_parameter("rotation_speed").value
        self._enabled_phases_str = self.get_parameter("enabled_phases").value
        self._interactive = self.get_parameter("interactive").value
        self._skip_preflight = self.get_parameter("skip_preflight").value

        # Parse enabled_phases from comma-separated string
        self._enabled_phases = self._parse_enabled_phases(self._enabled_phases_str)

        # ── Auto-detect config paths if not supplied ──────────────────
        if not self._slam_config_path:
            try:
                autonomy_share = get_package_share_directory("autonomy")
                self._slam_config_path = os.path.join(
                    autonomy_share, "config", "slam_toolbox_params_perseus_lite.yaml"
                )
            except Exception:
                self.get_logger().warn(
                    "Could not auto-detect SLAM config path. "
                    "Set slam_config_path parameter explicitly."
                )

        if not self._ekf_config_path:
            try:
                autonomy_share = get_package_share_directory("autonomy")
                self._ekf_config_path = os.path.join(
                    autonomy_share, "config", "ekf_params_perseus_lite.yaml"
                )
            except Exception:
                self.get_logger().warn(
                    "Could not auto-detect EKF config path. "
                    "Set ekf_config_path parameter explicitly."
                )

        # ── Load baseline configurations ──────────────────────────────
        self._baseline_slam = self._load_baseline_config(self._slam_config_path)
        self._baseline_ekf = self._load_baseline_config(self._ekf_config_path)

        # ── Auto-generate session name if empty ───────────────────────
        if not self._session_name:
            self._session_name = time.strftime("autotune_%Y%m%d_%H%M%S")

        # ── Initialize components ─────────────────────────────────────
        self._db = DbManager(db_path=self._db_path, logger=self.get_logger())

        defaults = {
            "imu_deadband_threshold": 0.01,
        }
        self._param_mgr = ParamManager(
            baseline_slam=self._baseline_slam,
            baseline_ekf=self._baseline_ekf,
            defaults=defaults,
        )
        self._analyzer = MapAnalyzer()
        self._maneuver = ManeuverExecutor(self)

        # ── Subscriptions ─────────────────────────────────────────────
        map_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )
        self._map_sub = self.create_subscription(
            OccupancyGrid, "/map", self._map_callback, map_qos
        )
        self._latest_map = None

        # ── State machine ─────────────────────────────────────────────
        if self._interactive:
            self._state = State.WAITING_FOR_TUI
        elif self._skip_preflight:
            self._state = State.CONFIGURING
        else:
            self._state = State.PREFLIGHT
        self._error_message = ""

        # Session / run tracking
        self._current_session_id = None
        self._current_run_id = None
        self._current_run_number = 0
        self._runs_completed = 0
        self._current_phase = 1
        self._phase_runs = []
        self._phase_run_index = 0

        # Best params accumulated across phases
        self._best_params = {}

        # Previous run params for restart detection (empty dict = baseline,
        # so the first run won't restart unless params actually differ)
        self._prev_slam_params = {}
        self._prev_ekf_params = {}
        self._restart_slam = False
        self._restart_ekf = False

        # Process handles for spawned nodes
        self._slam_process = None
        self._ekf_process = None
        self._imu_filter_process = None

        # Settling / resetting timing
        self._settling_start_time = None
        self._reset_start_time = None
        self._capture_start_time = None

        # Temp config file paths (for cleanup)
        self._current_slam_config_path = None
        self._current_ekf_config_path = None

        # Current run parameter dict
        self._current_run_params = None

        # Run allocation across phases
        self._phase_allocation = {}

        # Guard against re-entrant timer ticks (the maneuver handler blocks
        # for the full drive duration; without this lock the ReentrantCallbackGroup
        # allows the 1 Hz timer to fire again, spawning duplicate maneuvers).
        self._tick_lock = threading.Lock()

        # Callback group for the state machine timer
        self._cb_group = ReentrantCallbackGroup()
        self._state_machine_timer = self.create_timer(
            1.0, self._state_machine_tick, callback_group=self._cb_group
        )

        self.get_logger().info(
            f"AutotuneNode initialized. Session: {self._session_name}, "
            f"max_runs: {self._max_runs}"
        )

    # ══════════════════════════════════════════════════════════════════
    # Pre-flight checks
    # ══════════════════════════════════════════════════════════════════

    def _run_preflight(self) -> bool:
        """Run pre-flight checks and print formatted results.

        Returns:
            True if all critical checks pass.
        """
        results = []
        warnings = []

        self.get_logger().info("=== MAPPING AUTOTUNE PRE-FLIGHT ===")

        # 1. Database connection
        db_ok = self._db.check_connection()
        results.append(("Database connection", self._db_path, db_ok))

        # 2. SLAM node alive
        node_names = self.get_node_names_and_namespaces()
        slam_alive = any("slam_toolbox" in name for name, _ in node_names)
        results.append(("SLAM node (slam_toolbox)", None, slam_alive))

        # 3. EKF node alive
        ekf_alive = any("ekf_filter_node" in name for name, _ in node_names)
        results.append(("EKF node (ekf_filter_node)", None, ekf_alive))

        # 4. /map topic has publisher
        map_pub_count = self.count_publishers("/map")
        map_ok = map_pub_count > 0
        results.append(
            ("/map topic has publisher", f"{map_pub_count} publishers", map_ok)
        )

        # 5. /scan topic receiving data
        scan_msg = self._wait_for_topic_message("/scan", LaserScan, timeout=3.0)
        scan_ok = scan_msg is not None
        results.append(("/scan topic receiving", None, scan_ok))

        # 6. Odometry active
        odom_msg = self._wait_for_topic_message(
            "/diff_drive_base_controller/odom", Odometry, timeout=3.0
        )
        odom_ok = odom_msg is not None
        results.append(("/diff_drive_base_controller/odom active", None, odom_ok))

        # 7. IMU active (warning only)
        imu_msg = self._wait_for_topic_message("/imu/data", Imu, timeout=3.0)
        imu_ok = imu_msg is not None
        is_imu_warning = not imu_ok
        if is_imu_warning:
            warnings.append("/imu/data not receiving - IMU phases may fail")
        results.append(("/imu/data active", "warning only", imu_ok))

        # 8. /joy_vel writable
        joy_vel_ok = True
        try:
            pub = self.create_publisher(TwistStamped, "/joy_vel", 10)
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "base_link"
            pub.publish(msg)
            self.destroy_publisher(pub)
        except Exception as e:
            joy_vel_ok = False
            self.get_logger().error(f"/joy_vel publish test failed: {e}")
        results.append(("/joy_vel writable", None, joy_vel_ok))

        # 9. Robot stationary
        stationary_ok = True
        if odom_ok:
            odom_samples = []
            start = time.time()
            while time.time() - start < 1.0:
                sample = self._wait_for_topic_message(
                    "/diff_drive_base_controller/odom", Odometry, timeout=0.5
                )
                if sample is not None:
                    odom_samples.append(sample)
                    vx = abs(sample.twist.twist.linear.x)
                    vz = abs(sample.twist.twist.angular.z)
                    if vx > 0.05 or vz > 0.1:
                        stationary_ok = False
        else:
            stationary_ok = False
        results.append(("Robot stationary", None, stationary_ok))

        # 10. Disk space
        disk_ok = True
        disk_info = ""
        try:
            db_dir = os.path.dirname(self._db_path)
            if not os.path.isdir(db_dir):
                os.makedirs(db_dir, exist_ok=True)
            stat = os.statvfs(db_dir)
            free_mb = (stat.f_bavail * stat.f_frsize) / (1024 * 1024)
            disk_info = f"{free_mb:.0f} MB free"
            if free_mb < 100:
                disk_ok = False
                warnings.append(f"Low disk space: {free_mb:.0f} MB")
        except Exception as e:
            disk_ok = False
            disk_info = str(e)
        results.append(("Disk space", disk_info, disk_ok))

        # Print results
        passed = 0
        failed = 0
        warn_count = 0

        for name, detail, ok in results:
            detail_str = f" ({detail})" if detail else ""
            # The IMU check is a warning, not a failure
            is_warning = name == "/imu/data active" and not ok
            if ok:
                status = "[PASS]"
                passed += 1
            elif is_warning:
                status = "[WARN]"
                warn_count += 1
            else:
                status = "[FAIL]"
                failed += 1
            self.get_logger().info(f"{status} {name}{detail_str}")

        for w in warnings:
            self.get_logger().warn(w)

        self.get_logger().info(
            f"Pre-flight: {passed}/{len(results)} passed, "
            f"{warn_count} warning(s), {failed} failure(s)"
        )

        # Critical checks exclude the IMU warning
        critical_failures = sum(
            1 for name, _, ok in results if not ok and name != "/imu/data active"
        )
        return critical_failures == 0

    # ══════════════════════════════════════════════════════════════════
    # State machine
    # ══════════════════════════════════════════════════════════════════

    def _state_machine_tick(self):
        """Called at 1 Hz by the timer. Drives the state machine forward.

        Uses a non-blocking lock so that long-running handlers (preflight,
        maneuver execution) prevent the timer from re-entering and spawning
        duplicate work.
        """
        if not self._tick_lock.acquire(blocking=False):
            return  # Previous tick still running

        try:
            if self._state == State.WAITING_FOR_TUI:
                pass  # Blocked in main() until TUI completes
            elif self._state == State.PREFLIGHT:
                self._handle_preflight()
            elif self._state == State.IDLE:
                pass
            elif self._state == State.CONFIGURING:
                self._handle_configuring()
            elif self._state == State.RESETTING_SLAM:
                self._handle_resetting_slam()
            elif self._state == State.SETTLING:
                self._handle_settling()
            elif self._state == State.RUNNING_MANEUVER:
                self._handle_running_maneuver()
            elif self._state == State.CAPTURING_MAP:
                self._handle_capturing_map()
            elif self._state == State.ANALYZING:
                self._handle_analyzing()
            elif self._state == State.COMPLETE:
                self._handle_complete()
            elif self._state == State.ERROR:
                self._handle_error()
        except Exception as e:
            self.get_logger().error(f"State machine exception in {self._state}: {e}")
            self._error_message = str(e)
            self._state = State.ERROR
        finally:
            self._tick_lock.release()

    # ── State handlers ────────────────────────────────────────────────

    def _handle_preflight(self):
        """Run pre-flight checks and initialize the session."""
        if self._run_preflight():
            self._initialize_session()
            self._state = State.CONFIGURING
        else:
            self._error_message = "Pre-flight checks failed"
            self._state = State.ERROR

    def _initialize_session(self):
        """Create the DB session and generate the phase run plan."""
        self._current_session_id = self._db.create_session(
            name=self._session_name,
            description=self._session_description,
            base_slam_config=json.dumps(self._baseline_slam),
            base_ekf_config=json.dumps(self._baseline_ekf),
        )
        self.get_logger().info(
            f"Created session {self._current_session_id}: {self._session_name}"
        )

        # Allocate runs across phases
        self._phase_allocation = ParamManager.allocate_runs(
            self._max_runs, enabled_phases=self._enabled_phases
        )
        self.get_logger().info(f"Phase allocation: {self._phase_allocation}")

        # Find the first phase with allocated runs
        self._advance_to_next_phase(start_phase=1)

    def _advance_to_next_phase(self, start_phase=None):
        """Move to the next phase that has allocated runs.

        Args:
            start_phase: Phase number to start searching from. If None,
                         increments from _current_phase.
        """
        if start_phase is not None:
            phase = start_phase
        else:
            phase = self._current_phase + 1

        while phase <= 6:
            allocated = self._phase_allocation.get(phase, 0)
            if allocated > 0:
                self._current_phase = phase
                self._phase_runs = self._param_mgr.get_phase_runs(
                    phase, self._best_params
                )
                # Trim to allocated count
                self._phase_runs = self._phase_runs[:allocated]
                self._phase_run_index = 0
                self.get_logger().info(
                    f"Phase {phase}: {len(self._phase_runs)} runs planned"
                )
                return True
            phase += 1

        return False

    def _handle_configuring(self):
        """Pick the next run parameters and prepare config files."""
        # Check if we have runs remaining
        if not self._phase_runs or self._phase_run_index >= len(self._phase_runs):
            # Phase complete -- find best from this phase and advance
            if not self._finish_phase_and_advance():
                self._state = State.COMPLETE
                return
            # If advance succeeded, continue with new phase runs
            if not self._phase_runs or self._phase_run_index >= len(self._phase_runs):
                self._state = State.COMPLETE
                return

        # Check global run budget
        if self._runs_completed >= self._max_runs:
            self.get_logger().info("Max runs reached.")
            self._state = State.COMPLETE
            return

        # Get current run parameters
        self._current_run_params = self._phase_runs[self._phase_run_index]
        self._current_run_number += 1

        run_label = self._current_run_params.get(
            "run_label", f"run_{self._current_run_number}"
        )
        self.get_logger().info(
            f"Configuring run {self._current_run_number} "
            f"(phase {self._current_phase}, {run_label})"
        )

        # Create DB run record
        self._current_run_id = self._db.create_run(
            session_id=self._current_session_id,
            run_number=self._current_run_number,
            slam_params=json.dumps(self._current_run_params.get("slam", {})),
            ekf_params=json.dumps(self._current_run_params.get("ekf", {})),
            imu_params=json.dumps(self._current_run_params.get("imu_filter", {})),
            maneuver_params=json.dumps(self._current_run_params.get("maneuver", {})),
        )

        # Write temp config files
        self._current_slam_config_path = self._param_mgr.apply_slam_params(
            self._current_run_params
        )
        self._current_ekf_config_path = self._param_mgr.apply_ekf_params(
            self._current_run_params
        )

        # Determine which processes need a restart
        current_slam = self._current_run_params.get("slam", {})
        current_ekf = self._current_run_params.get("ekf", {})

        self._restart_slam = current_slam != self._prev_slam_params
        self._restart_ekf = current_ekf != self._prev_ekf_params

        self._prev_slam_params = current_slam
        self._prev_ekf_params = current_ekf

        if self._restart_slam or self._restart_ekf:
            self._reset_start_time = None
            self._state = State.RESETTING_SLAM
        else:
            self._settling_start_time = None
            self._state = State.SETTLING

    def _handle_resetting_slam(self):
        """Kill and relaunch only the processes whose params changed."""
        if self._reset_start_time is None:
            self._reset_start_time = time.time()

            if self._restart_slam:
                self.get_logger().info("Killing existing SLAM process...")
                try:
                    subprocess.run(
                        ["pkill", "-f", "async_slam_toolbox_node"],
                        timeout=5,
                        capture_output=True,
                    )
                except Exception as e:
                    self.get_logger().warn(f"pkill slam failed: {e}")
                self._terminate_process(self._slam_process)
                self._slam_process = None

            if self._restart_ekf:
                self.get_logger().info("Killing existing EKF process...")
                try:
                    subprocess.run(
                        ["pkill", "-f", "ekf_node"], timeout=5, capture_output=True
                    )
                except Exception as e:
                    self.get_logger().warn(f"pkill ekf failed: {e}")
                self._terminate_process(self._ekf_process)
                self._ekf_process = None

            return  # Wait one tick for cleanup

        elapsed = time.time() - self._reset_start_time
        relaunch_ready = elapsed >= 2.0

        # Relaunch SLAM if needed
        if relaunch_ready and self._restart_slam and self._slam_process is None:
            self.get_logger().info("Relaunching SLAM with new parameters...")
            try:
                self._slam_process = subprocess.Popen(
                    [
                        "ros2",
                        "run",
                        "slam_toolbox",
                        "async_slam_toolbox_node",
                        "--ros-args",
                        "-r",
                        "__node:=slam_toolbox",
                        "-r",
                        "__ns:=/",
                        "--params-file",
                        self._current_slam_config_path,
                    ],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                )
            except Exception as e:
                self.get_logger().error(f"Failed to launch SLAM: {e}")
                self._error_message = f"SLAM launch failed: {e}"
                self._state = State.ERROR
                return

        # Relaunch EKF if needed
        if relaunch_ready and self._restart_ekf and self._ekf_process is None:
            self.get_logger().info("Relaunching EKF with new parameters...")
            try:
                self._ekf_process = subprocess.Popen(
                    [
                        "ros2",
                        "run",
                        "robot_localization",
                        "ekf_node",
                        "--ros-args",
                        "-r",
                        "__node:=ekf_filter_node",
                        "--params-file",
                        self._current_ekf_config_path,
                    ],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                )
            except Exception as e:
                self.get_logger().error(f"Failed to launch EKF: {e}")
                self._error_message = f"EKF launch failed: {e}"
                self._state = State.ERROR
                return

        # Wait for /map to have a publisher (either the original or relaunched)
        if elapsed >= 3.0:
            map_pubs = self.count_publishers("/map")
            if map_pubs > 0:
                self.get_logger().info("/map publisher detected. Moving to settling.")
                self._settling_start_time = None
                self._state = State.SETTLING
                return

        # Timeout
        if elapsed > 15.0:
            self.get_logger().error("Timeout waiting for SLAM to start publishing /map")
            self._error_message = "SLAM restart timeout (no /map publisher after 15s)"
            self._state = State.ERROR

    def _handle_settling(self):
        """Wait for settling_time seconds after SLAM restart."""
        if self._settling_start_time is None:
            self._settling_start_time = time.time()
            self.get_logger().info(f"Settling for {self._settling_time:.1f}s...")
            return

        elapsed = time.time() - self._settling_start_time
        if elapsed >= self._settling_time:
            self._db.update_run_status(self._current_run_id, "running")
            self._state = State.RUNNING_MANEUVER

    def _handle_running_maneuver(self):
        """Execute the maneuver in the current thread (timer callback).

        The MultiThreadedExecutor ensures the node remains responsive to
        subscriptions (map, odom) while this callback blocks.
        """
        run_label = self._current_run_params.get(
            "run_label", f"run_{self._current_run_number}"
        )
        self.get_logger().info(f"Starting maneuver for {run_label}...")

        # Prepare maneuver params — use node-level defaults, allow per-run overrides
        maneuver_params = dict(self._current_run_params.get("maneuver", {}))
        maneuver_params.setdefault("pattern", self._maneuver_pattern)
        maneuver_params.setdefault("linear_speed", self._linear_speed)
        maneuver_params.setdefault("rotation_speed", self._rotation_speed)

        # Start sensor logging
        self._maneuver.start_logging()

        # Execute maneuver (blocking)
        success = self._maneuver.execute_maneuver(maneuver_params)

        # Stop logging and save
        odom_json, imu_json = self._maneuver.stop_logging()
        self._db.store_sensor_logs(self._current_run_id, odom_json, imu_json)

        if not success:
            self.get_logger().warn(f"Maneuver failed for {run_label}")
            self._db.update_run_status(self._current_run_id, "failed")
            # Still try to capture and analyze the map
        else:
            self.get_logger().info(f"Maneuver completed for {run_label}")

        self._capture_start_time = None
        self._state = State.CAPTURING_MAP

    def _handle_capturing_map(self):
        """Wait briefly for the final map update, then store map data."""
        if self._capture_start_time is None:
            self._capture_start_time = time.time()
            return

        elapsed = time.time() - self._capture_start_time
        if elapsed < 2.0:
            return

        if self._latest_map is None:
            self.get_logger().warn("No map data available for capture.")
            self._db.update_run_status(self._current_run_id, "failed")
            self._phase_run_index += 1
            self._runs_completed += 1
            self._state = State.CONFIGURING
            return

        map_msg = self._latest_map
        width = map_msg.info.width
        height = map_msg.info.height
        resolution = map_msg.info.resolution
        raw_data = list(map_msg.data)

        # Compress map data
        compressed = self._analyzer.compress_map(raw_data)

        # Generate PNG thumbnail
        try:
            png_bytes = self._analyzer.map_to_png(raw_data, width, height)
        except Exception as e:
            self.get_logger().warn(f"PNG generation failed: {e}")
            png_bytes = None

        # Store in DB
        self._db.store_map_data(
            self._current_run_id,
            compressed,
            width,
            height,
            resolution,
            map_png=png_bytes,
        )

        self._state = State.ANALYZING

    def _handle_analyzing(self):
        """Run map quality analysis and store results."""
        if self._latest_map is None:
            self.get_logger().warn("No map data for analysis.")
            self._db.update_run_status(self._current_run_id, "failed")
            self._phase_run_index += 1
            self._runs_completed += 1
            self._state = State.CONFIGURING
            return

        map_msg = self._latest_map
        width = map_msg.info.width
        height = map_msg.info.height
        resolution = map_msg.info.resolution
        raw_data = list(map_msg.data)

        # Run analysis
        metrics = self._analyzer.analyze(raw_data, width, height, resolution)

        # Serialize diagnostics to JSON string for DB storage
        diagnostics = metrics.pop("diagnostics", {})
        metrics["diagnostics"] = json.dumps(diagnostics)

        # Store analysis in DB
        self._db.store_analysis(self._current_run_id, metrics)

        # Update run status
        completed_at = time.strftime("%Y-%m-%d %H:%M:%S")
        self._db.update_run_status(
            self._current_run_id, "completed", completed_at=completed_at
        )

        # Log results
        run_label = self._current_run_params.get(
            "run_label", f"run_{self._current_run_number}"
        )
        composite = metrics.get("composite_score", 0.0)
        wall_str = metrics.get("wall_straightness", 0.0)
        thickness = metrics.get("wall_thickness", 0.0)
        ghost = metrics.get("ghost_wall_score", 0.0)

        self.get_logger().info(
            f"Run {self._current_run_number} ({run_label}): "
            f"composite_score={composite:.4f} "
            f"(wall={wall_str:.4f}, thickness={thickness:.4f}, "
            f"ghost={ghost:.4f})"
        )

        # Advance counters
        self._phase_run_index += 1
        self._runs_completed += 1

        # Check if more runs in current phase
        if self._phase_run_index < len(self._phase_runs):
            self._state = State.CONFIGURING
        else:
            # Phase complete
            if not self._finish_phase_and_advance():
                self._state = State.COMPLETE
            else:
                self._state = State.CONFIGURING

    def _finish_phase_and_advance(self) -> bool:
        """Find best run in current phase, update best_params, advance phase.

        Returns:
            True if a new phase was started, False if all phases done.
        """
        # Find best run from current session
        best_run = self._db.get_best_run(self._current_session_id)
        if best_run is not None:
            try:
                best_slam = json.loads(best_run.get("slam_params", "{}"))
            except (json.JSONDecodeError, TypeError):
                best_slam = {}
            try:
                best_ekf = json.loads(best_run.get("ekf_params", "{}"))
            except (json.JSONDecodeError, TypeError):
                best_ekf = {}

            self._best_params = {
                "slam": best_slam,
                "ekf": best_ekf,
            }
            self.get_logger().info(
                f"Phase {self._current_phase} complete. "
                f"Best composite: {best_run.get('composite_score', 0.0):.4f} "
                f"from run {best_run.get('run_number', '?')}"
            )

        # Check budget
        if self._runs_completed >= self._max_runs:
            self.get_logger().info("Run budget exhausted.")
            return False

        # Try to advance to next phase
        return self._advance_to_next_phase()

    def _handle_complete(self):
        """Finalize the session: update status, sync, export, log summary."""
        if self._current_session_id is not None:
            self._db.update_session_status(self._current_session_id, "completed")

            # Sync to remote
            try:
                self._db.sync_to_remote()
            except Exception as e:
                self.get_logger().warn(f"Remote sync failed: {e}")

            # Export report
            try:
                report_path = self._db.export_session_report(self._current_session_id)
                self.get_logger().info(f"Report exported to {report_path}")
            except Exception as e:
                self.get_logger().warn(f"Report export failed: {e}")

            # Log summary
            best_run = self._db.get_best_run(self._current_session_id)
            if best_run is not None:
                self.get_logger().info(
                    f"Session complete. Best composite score: "
                    f"{best_run.get('composite_score', 0.0):.4f} "
                    f"from run {best_run.get('run_number', '?')}"
                )
            else:
                self.get_logger().info("Session complete. No successful runs.")

            self.get_logger().info(
                "Use `ros2 run mapping_autotune review_tui` to review results"
            )

        # Cancel the timer to stop ticking
        self._state_machine_timer.cancel()
        self._state = State.IDLE

    def _handle_error(self):
        """Log the error, abort session, and stop."""
        self.get_logger().error(f"Autotune error: {self._error_message}")

        if self._current_session_id is not None:
            try:
                self._db.update_session_status(self._current_session_id, "aborted")
            except Exception:
                pass

        if self._current_run_id is not None:
            try:
                self._db.update_run_status(self._current_run_id, "failed")
            except Exception:
                pass

        self._state_machine_timer.cancel()
        self._state = State.IDLE

    # ══════════════════════════════════════════════════════════════════
    # Helpers
    # ══════════════════════════════════════════════════════════════════

    def _wait_for_topic_message(self, topic, msg_type, timeout=3.0):
        """Subscribe temporarily and wait for a single message.

        Args:
            topic: Topic name string.
            msg_type: ROS message type class.
            timeout: Maximum seconds to wait.

        Returns:
            The received message, or None on timeout.
        """
        received = {"msg": None}
        event = threading.Event()

        def _callback(msg):
            received["msg"] = msg
            event.set()

        sub = self.create_subscription(msg_type, topic, _callback, 10)
        try:
            event.wait(timeout=timeout)
        finally:
            self.destroy_subscription(sub)

        return received["msg"]

    def _load_baseline_config(self, config_path) -> dict:
        """Load a YAML config file and return the full dict.

        Args:
            config_path: Path to the YAML file.

        Returns:
            The parsed YAML dict, or an empty dict on failure.
        """
        if not config_path or not os.path.isfile(config_path):
            self.get_logger().warn(f"Config file not found: {config_path}")
            return {}

        try:
            with open(config_path, "r") as f:
                data = yaml.safe_load(f) or {}
            self.get_logger().info(f"Loaded baseline config: {config_path}")
            return data
        except Exception as e:
            self.get_logger().error(f"Failed to load config {config_path}: {e}")
            return {}

    def _map_callback(self, msg):
        """Store the latest OccupancyGrid message."""
        self._latest_map = msg

    def _terminate_process(self, proc):
        """Gracefully terminate a subprocess, escalating to SIGKILL.

        Args:
            proc: A subprocess.Popen instance, or None.
        """
        if proc is None:
            return
        try:
            proc.send_signal(signal.SIGTERM)
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()
            try:
                proc.wait(timeout=3)
            except Exception:
                pass
        except Exception:
            pass

    @staticmethod
    def _parse_enabled_phases(phases_str):
        """Parse a comma-separated string of phase numbers.

        Args:
            phases_str: Comma-separated phase numbers (e.g. "1,2,3,4").
                Empty string means all phases.

        Returns:
            List of phase ints, or None if empty (meaning all phases).
        """
        if not phases_str or not phases_str.strip():
            return None
        try:
            return [int(p.strip()) for p in phases_str.split(",") if p.strip()]
        except ValueError:
            return None

    def apply_tui_config(self, config):
        """Apply configuration from the setup TUI to node parameters.

        Args:
            config: Dict returned by SetupTUI.run().
        """
        self._session_name = config.get("session_name", self._session_name)
        self._max_runs = config.get("max_runs", self._max_runs)
        self._settling_time = config.get("settling_time", self._settling_time)
        self._maneuver_pattern = config.get("maneuver_pattern", self._maneuver_pattern)
        self._linear_speed = config.get("linear_speed", self._linear_speed)
        self._rotation_speed = config.get("rotation_speed", self._rotation_speed)

        enabled = config.get("enabled_phases")
        if enabled:
            self._enabled_phases = enabled
        else:
            self._enabled_phases = None

        self.get_logger().info(
            f"TUI config applied: session={self._session_name}, "
            f"max_runs={self._max_runs}, pattern={self._maneuver_pattern}, "
            f"linear={self._linear_speed}, rotation={self._rotation_speed}"
        )

    def cleanup(self):
        """Kill any spawned SLAM/EKF/IMU filter processes gracefully."""
        self.get_logger().info("Cleaning up spawned processes...")
        self._terminate_process(self._slam_process)
        self._slam_process = None
        self._terminate_process(self._ekf_process)
        self._ekf_process = None
        self._terminate_process(self._imu_filter_process)
        self._imu_filter_process = None

        # Clean up temp config files
        for path in (self._current_slam_config_path, self._current_ekf_config_path):
            if path and os.path.isfile(path):
                try:
                    os.unlink(path)
                except Exception:
                    pass


def main(args=None):
    rclpy.init(args=args)
    node = AutotuneNode()

    # If interactive mode, run the setup TUI before spinning
    if node._interactive:
        from mapping_autotune.setup_tui import SetupTUI

        defaults = {
            "session_name": node._session_name,
            "max_runs": node._max_runs,
            "settling_time": node._settling_time,
            "maneuver_pattern": node._maneuver_pattern,
            "linear_speed": node._linear_speed,
            "rotation_speed": node._rotation_speed,
        }

        tui = SetupTUI(defaults=defaults)
        config = tui.run()

        if config is None:
            node.get_logger().info("Setup cancelled by user.")
            node.destroy_node()
            rclpy.shutdown()
            return

        node.apply_tui_config(config)

        # Transition past WAITING_FOR_TUI
        if node._skip_preflight:
            node._state = State.CONFIGURING
        else:
            node._state = State.PREFLIGHT

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
