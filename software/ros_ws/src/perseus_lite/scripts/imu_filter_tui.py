#!/usr/bin/env python3
"""IMU filter node with interactive curses TUI + F-16 HUD for Perseus Lite.

Subscribes to raw IMU and wheel odometry, applies bias subtraction and deadband
filtering to angular velocity, publishes filtered IMU, and displays a live TUI
showing raw vs filtered values with sparkline history and an F-16 style HUD.

Interactive controls let the user manually tune bias and deadband in real time,
recalibrate, and save settings to a YAML file for use in launch files.

Usage:
    python3 imu_filter_tui.py
    # or with ROS args:
    python3 imu_filter_tui.py --ros-args -p calibration_duration:=10.0
"""

import collections
import curses
import math
import os
import threading
import time

import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry


# Braille sparkline characters (8 levels)
_SPARKS = " \u2581\u2582\u2583\u2584\u2585\u2586\u2587\u2588"

# Save path for tuned settings
_DEFAULT_SAVE_PATH = os.path.expanduser(
    "~/perseus-v2/software/ros_ws/src/perseus_lite/config/imu_filter_params.yaml"
)

# Tunable parameters and their step sizes
_TUNABLES = [
    ("bias_x", 0.0001),
    ("bias_y", 0.0001),
    ("bias_z", 0.0001),
    ("deadband", 0.001),
    ("stationary_vx", 0.005),
    ("stationary_vyaw", 0.005),
]

# ──────────────────────────────────────────────────────────────
# HUD constants
# ──────────────────────────────────────────────────────────────
_HUD_W = 35  # character width of the HUD viewport
_HUD_H = 17  # character height of the HUD viewport
_HUD_CX = _HUD_W // 2  # centre column
_HUD_CY = _HUD_H // 2  # centre row (horizon line at rest)
_PITCH_SCALE = 0.6  # rows per degree of pitch
_MAX_VEL_BAR = 2.0  # m/s for full velocity bar


def _sparkline(values, width=30):
    """Render a list of floats as a braille sparkline string."""
    if not values:
        return " " * width
    recent = list(values)[-width:]
    lo = min(recent)
    hi = max(recent)
    span = hi - lo if hi != lo else 1.0
    out = []
    for v in recent:
        idx = int((v - lo) / span * (len(_SPARKS) - 1))
        idx = max(0, min(idx, len(_SPARKS) - 1))
        out.append(_SPARKS[idx])
    return "".join(out).ljust(width)


def _render_hud(pitch_deg, roll_deg, yaw_rate, velocity):
    """Render an F-16 style HUD into a list of (text, color_id) rows.

    Returns a list of length _HUD_H.  Each element is a list of
    (col, string, color_pair_id) tuples to draw on that row.
    """
    # We build a character grid then layer things on top.
    # Color map: 0=border, 1=horizon, 2=pitch_ladder, 3=boresight,
    #            4=text_readout, 5=sky, 6=ground
    grid = [[" "] * _HUD_W for _ in range(_HUD_H)]
    color = [[0] * _HUD_W for _ in range(_HUD_H)]

    # --- Border ---
    for c in range(_HUD_W):
        grid[0][c] = "\u2550"  # ═
        grid[_HUD_H - 1][c] = "\u2550"
        color[0][c] = 10
        color[_HUD_H - 1][c] = 10
    for r in range(_HUD_H):
        grid[r][0] = "\u2551"  # ║
        grid[r][_HUD_W - 1] = "\u2551"
        color[r][0] = 10
        color[r][_HUD_W - 1] = 10
    grid[0][0] = "\u2554"  # ╔
    grid[0][_HUD_W - 1] = "\u2557"  # ╗
    grid[_HUD_H - 1][0] = "\u255a"  # ╚
    grid[_HUD_H - 1][_HUD_W - 1] = "\u255d"  # ╝

    # Title
    title = " HUD "
    ts = max(1, _HUD_CX - len(title) // 2)
    for i, ch in enumerate(title):
        if ts + i < _HUD_W - 1:
            grid[0][ts + i] = ch
            color[0][ts + i] = 10

    inner_left = 2
    inner_right = _HUD_W - 3
    inner_top = 1
    inner_bottom = _HUD_H - 2
    inner_w = inner_right - inner_left + 1

    # --- Horizon line (shifted by pitch, tilted by roll) ---
    # Pitch moves horizon: positive pitch (nose up) -> horizon moves DOWN
    pitch_offset = pitch_deg * _PITCH_SCALE

    # Draw horizon with roll tilt
    roll_rad = math.radians(roll_deg)
    for c in range(inner_left, inner_right + 1):
        dx = c - _HUD_CX
        # y offset from centre due to roll tilt
        dy = dx * math.tan(roll_rad)
        r = int(round(_HUD_CY + pitch_offset + dy))
        if inner_top <= r <= inner_bottom:
            grid[r][c] = "\u2550"  # ═
            color[r][c] = 11  # horizon color

    # --- Pitch ladder lines ---
    for angle in [-20, -15, -10, -5, 5, 10, 15, 20]:
        # Position relative to horizon
        ladder_y_offset = -angle * _PITCH_SCALE  # negative angle = above horizon
        r_center = int(round(_HUD_CY + pitch_offset + ladder_y_offset))

        if not (inner_top + 1 <= r_center <= inner_bottom - 1):
            continue

        # Draw short dashes with angle label
        label = f"{abs(angle):d}"
        half_w = 4  # half-width of ladder line in chars

        # Dashed line segments
        dash_char = "\u2500" if angle < 0 else "\u2504"  # ─ solid for nose up, ┄ dashed for nose down
        for dc in range(-half_w, half_w + 1):
            c = _HUD_CX + dc
            # Apply roll tilt
            dy = dc * math.tan(roll_rad)
            r = int(round(r_center + dy))
            if inner_left <= c <= inner_right and inner_top <= r <= inner_bottom:
                if dc == -half_w or dc == half_w:
                    # Tick marks: up for positive pitch, down for negative
                    tick_r = r - 1 if angle > 0 else r + 1
                    if inner_top <= tick_r <= inner_bottom:
                        grid[tick_r][c] = "\u2502"  # │
                        color[tick_r][c] = 12
                elif abs(dc) <= half_w:
                    grid[r][c] = dash_char
                    color[r][c] = 12

        # Labels on each side
        for lc, txt in [(_HUD_CX - half_w - len(label) - 1, label),
                        (_HUD_CX + half_w + 2, label)]:
            for i, ch in enumerate(txt):
                c = lc + i
                if inner_left <= c <= inner_right:
                    dy = (c - _HUD_CX) * math.tan(roll_rad)
                    r = int(round(r_center + dy))
                    if inner_top <= r <= inner_bottom:
                        grid[r][c] = ch
                        color[r][c] = 12

    # --- Boresight / Flight Path Marker (always at centre) ---
    # Classic F-16 FPM:  -◇-
    fpm_chars = [("\u25c1", 13), ("\u2500", 13), ("\u25c7", 14), ("\u2500", 13), ("\u25b7", 13)]
    for i, (ch, col) in enumerate(fpm_chars):
        c = _HUD_CX - 2 + i
        if inner_left <= c <= inner_right:
            grid[_HUD_CY][c] = ch
            color[_HUD_CY][c] = col

    # --- Readouts bar at bottom of HUD ---
    # Pitch readout (left)
    pitch_str = f"P{pitch_deg:+5.1f}\u00b0"
    for i, ch in enumerate(pitch_str):
        c = inner_left + i
        if c <= inner_right:
            grid[inner_bottom][c] = ch
            color[inner_bottom][c] = 15

    # Roll readout (right)
    roll_str = f"R{roll_deg:+5.1f}\u00b0"
    start_c = inner_right - len(roll_str) + 1
    for i, ch in enumerate(roll_str):
        c = start_c + i
        if inner_left <= c <= inner_right:
            grid[inner_bottom][c] = ch
            color[inner_bottom][c] = 15

    # --- Yaw rate tape (row below HUD, rendered separately) ---
    # --- Velocity indicator (row below yaw tape) ---

    # Convert grid to row-segments for the TUI
    rows = []
    for r in range(_HUD_H):
        rows.append("".join(grid[r]))
    colors = []
    for r in range(_HUD_H):
        colors.append(list(color[r]))

    return rows, colors, pitch_deg, roll_deg, yaw_rate, velocity


class ImuFilterTuiNode(Node):
    """ROS 2 node: IMU bias filter with stationary detection."""

    def __init__(self):
        super().__init__("imu_filter_tui")

        # Parameters
        self.declare_parameter("calibration_duration", 5.0)
        self.declare_parameter("gyro_deadband", 0.01)
        self.declare_parameter("stationary_vx_thresh", 0.01)
        self.declare_parameter("stationary_vyaw_thresh", 0.02)

        # State
        self._lock = threading.Lock()
        self._is_calibrating = True
        self._cal_start = None
        self._cal_samples_x = []
        self._cal_samples_y = []
        self._cal_samples_z = []
        self._bias = [0.0, 0.0, 0.0]
        self._cal_stddev = [0.0, 0.0, 0.0]
        self._deadband = self.get_parameter("gyro_deadband").value
        self._stationary_vx_thresh = self.get_parameter("stationary_vx_thresh").value
        self._stationary_vyaw_thresh = self.get_parameter("stationary_vyaw_thresh").value

        # Latest values for TUI
        self._raw_gyro = [0.0, 0.0, 0.0]
        self._filt_gyro = [0.0, 0.0, 0.0]
        self._raw_accel = [0.0, 0.0, 0.0]
        self._filt_accel = [0.0, 0.0, 0.0]
        self._filter_labels = ["---", "---", "---"]
        self._accel_labels = ["PASS", "PASS", "PASS"]
        self._odom_vx = 0.0
        self._odom_vyaw = 0.0
        self._is_stationary = True
        self._imu_count = 0
        self._imu_rate = 0.0
        self._rate_ts = time.monotonic()
        self._rate_count = 0

        # Derived attitude (from filtered accel)
        self._pitch_deg = 0.0
        self._roll_deg = 0.0

        # History for sparklines (last 60 samples)
        self._hist_raw_gx = collections.deque(maxlen=60)
        self._hist_raw_gy = collections.deque(maxlen=60)
        self._hist_raw_gz = collections.deque(maxlen=60)
        self._hist_filt_gx = collections.deque(maxlen=60)
        self._hist_filt_gy = collections.deque(maxlen=60)
        self._hist_filt_gz = collections.deque(maxlen=60)
        self._hist_ax = collections.deque(maxlen=60)
        self._hist_ay = collections.deque(maxlen=60)
        self._hist_az = collections.deque(maxlen=60)

        # Publisher
        self._pub = self.create_publisher(Imu, "/imu/data_filtered", 10)

        # Subscribers
        best_effort = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._sub_imu = self.create_subscription(
            Imu, "/imu/data", self._imu_cb, best_effort
        )
        self._sub_odom = self.create_subscription(
            Odometry,
            "/diff_drive_base_controller/odom",
            self._odom_cb,
            best_effort,
        )

    def _odom_cb(self, msg: Odometry):
        with self._lock:
            self._odom_vx = msg.twist.twist.linear.x
            self._odom_vyaw = msg.twist.twist.angular.z
            self._is_stationary = (
                abs(self._odom_vx) < self._stationary_vx_thresh
                and abs(self._odom_vyaw) < self._stationary_vyaw_thresh
            )

    def _imu_cb(self, msg: Imu):
        gx = msg.angular_velocity.x
        gy = msg.angular_velocity.y
        gz = msg.angular_velocity.z
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z

        with self._lock:
            self._raw_gyro = [gx, gy, gz]
            self._raw_accel = [ax, ay, az]

            # Derive pitch/roll from accelerometer
            g_mag = math.sqrt(ax * ax + ay * ay + az * az)
            if g_mag > 0.1:
                self._pitch_deg = math.degrees(math.atan2(-ax, math.sqrt(ay * ay + az * az)))
                self._roll_deg = math.degrees(math.atan2(ay, az))

            # Rate calculation
            self._rate_count += 1
            now = time.monotonic()
            dt = now - self._rate_ts
            if dt >= 1.0:
                self._imu_rate = self._rate_count / dt
                self._rate_count = 0
                self._rate_ts = now

            # Calibration phase
            if self._is_calibrating:
                if self._cal_start is None:
                    self._cal_start = now
                self._cal_samples_x.append(gx)
                self._cal_samples_y.append(gy)
                self._cal_samples_z.append(gz)
                self._imu_count += 1

                cal_dur = self.get_parameter("calibration_duration").value
                if now - self._cal_start >= cal_dur:
                    self._finish_calibration()
                else:
                    self._filt_gyro = [gx, gy, gz]
                    self._filt_accel = [ax, ay, az]
                    self._filter_labels = ["CAL", "CAL", "CAL"]
                    self._accel_labels = ["CAL", "CAL", "CAL"]
                    self._update_history()
                    self._publish_filtered(msg, gx, gy, gz, ax, ay, az)
                    return

            self._imu_count += 1

            # Bias subtraction
            cx = gx - self._bias[0]
            cy = gy - self._bias[1]
            cz = gz - self._bias[2]

            labels = ["BIAS", "BIAS", "BIAS"]

            # Deadband filter
            mag = math.sqrt(cx * cx + cy * cy + cz * cz)
            if mag < self._deadband:
                cx, cy, cz = 0.0, 0.0, 0.0
                labels = ["BIAS+DB", "BIAS+DB", "BIAS+DB"]

            # Extra stationary zeroing
            if self._is_stationary and mag < self._deadband * 3.0:
                cx, cy, cz = 0.0, 0.0, 0.0
                labels = ["STATIC", "STATIC", "STATIC"]

            self._filt_gyro = [cx, cy, cz]
            self._filt_accel = [ax, ay, az]
            self._filter_labels = labels
            self._accel_labels = ["PASS", "PASS", "PASS"]

            self._update_history()

        self._publish_filtered(msg, cx, cy, cz, ax, ay, az)

    def _finish_calibration(self):
        """Compute bias from collected samples."""
        import numpy as np

        sx = np.array(self._cal_samples_x)
        sy = np.array(self._cal_samples_y)
        sz = np.array(self._cal_samples_z)

        self._bias = [float(np.mean(sx)), float(np.mean(sy)), float(np.mean(sz))]
        self._cal_stddev = [float(np.std(sx)), float(np.std(sy)), float(np.std(sz))]

        auto_db = 3.0 * max(self._cal_stddev)
        if auto_db > self._deadband:
            self._deadband = auto_db

        self._cal_samples_x.clear()
        self._cal_samples_y.clear()
        self._cal_samples_z.clear()
        self._is_calibrating = False

        self.get_logger().info(
            f"Calibration done: bias=[{self._bias[0]:.6f}, {self._bias[1]:.6f}, "
            f"{self._bias[2]:.6f}] deadband={self._deadband:.6f}"
        )

    def _update_history(self):
        self._hist_raw_gx.append(self._raw_gyro[0])
        self._hist_raw_gy.append(self._raw_gyro[1])
        self._hist_raw_gz.append(self._raw_gyro[2])
        self._hist_filt_gx.append(self._filt_gyro[0])
        self._hist_filt_gy.append(self._filt_gyro[1])
        self._hist_filt_gz.append(self._filt_gyro[2])
        self._hist_ax.append(self._raw_accel[0])
        self._hist_ay.append(self._raw_accel[1])
        self._hist_az.append(self._raw_accel[2])

    def _publish_filtered(self, orig, gx, gy, gz, ax, ay, az):
        out = Imu()
        out.header = orig.header
        out.orientation = orig.orientation
        out.orientation_covariance = orig.orientation_covariance
        out.angular_velocity.x = gx
        out.angular_velocity.y = gy
        out.angular_velocity.z = gz
        out.angular_velocity_covariance = orig.angular_velocity_covariance
        out.linear_acceleration.x = ax
        out.linear_acceleration.y = ay
        out.linear_acceleration.z = az
        out.linear_acceleration_covariance = orig.linear_acceleration_covariance
        self._pub.publish(out)

    # --- Interactive tuning API ---

    def adjust_bias(self, axis, delta):
        with self._lock:
            self._bias[axis] += delta

    def adjust_deadband(self, delta):
        with self._lock:
            self._deadband = max(0.0, self._deadband + delta)

    def adjust_stationary_vx(self, delta):
        with self._lock:
            self._stationary_vx_thresh = max(0.0, self._stationary_vx_thresh + delta)

    def adjust_stationary_vyaw(self, delta):
        with self._lock:
            self._stationary_vyaw_thresh = max(0.0, self._stationary_vyaw_thresh + delta)

    def recalibrate(self):
        with self._lock:
            self._is_calibrating = True
            self._cal_start = None
            self._cal_samples_x.clear()
            self._cal_samples_y.clear()
            self._cal_samples_z.clear()

    def save_params(self, path=None):
        path = path or _DEFAULT_SAVE_PATH
        with self._lock:
            params = {
                "imu_filter_tui": {
                    "ros__parameters": {
                        "gyro_bias_x": round(self._bias[0], 8),
                        "gyro_bias_y": round(self._bias[1], 8),
                        "gyro_bias_z": round(self._bias[2], 8),
                        "gyro_deadband": round(self._deadband, 8),
                        "stationary_vx_thresh": round(self._stationary_vx_thresh, 6),
                        "stationary_vyaw_thresh": round(self._stationary_vyaw_thresh, 6),
                    }
                }
            }
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w") as f:
            yaml.dump(params, f, default_flow_style=False, sort_keys=False)
        return path

    def get_state(self):
        with self._lock:
            return {
                "calibrating": self._is_calibrating,
                "cal_progress": self._cal_progress(),
                "bias": list(self._bias),
                "stddev": list(self._cal_stddev),
                "deadband": self._deadband,
                "stationary_vx_thresh": self._stationary_vx_thresh,
                "stationary_vyaw_thresh": self._stationary_vyaw_thresh,
                "raw_gyro": list(self._raw_gyro),
                "filt_gyro": list(self._filt_gyro),
                "raw_accel": list(self._raw_accel),
                "filt_accel": list(self._filt_accel),
                "filter_labels": list(self._filter_labels),
                "accel_labels": list(self._accel_labels),
                "odom_vx": self._odom_vx,
                "odom_vyaw": self._odom_vyaw,
                "stationary": self._is_stationary,
                "imu_count": self._imu_count,
                "imu_rate": self._imu_rate,
                "pitch_deg": self._pitch_deg,
                "roll_deg": self._roll_deg,
                "hist_raw_gx": self._hist_raw_gx,
                "hist_raw_gy": self._hist_raw_gy,
                "hist_raw_gz": self._hist_raw_gz,
                "hist_filt_gx": self._hist_filt_gx,
                "hist_filt_gy": self._hist_filt_gy,
                "hist_filt_gz": self._hist_filt_gz,
                "hist_ax": self._hist_ax,
                "hist_ay": self._hist_ay,
                "hist_az": self._hist_az,
            }

    def _cal_progress(self):
        if not self._is_calibrating or self._cal_start is None:
            return 1.0
        elapsed = time.monotonic() - self._cal_start
        dur = self.get_parameter("calibration_duration").value
        return min(elapsed / dur, 1.0)


# ──────────────────────────────────────────────────────────────
# TUI drawing
# ──────────────────────────────────────────────────────────────

def _draw_tui(stdscr, node: ImuFilterTuiNode):
    """Main curses draw loop with interactive tuning and HUD."""
    curses.curs_set(0)
    stdscr.nodelay(True)

    curses.start_color()
    curses.use_default_colors()
    # Data panel colors
    curses.init_pair(1, curses.COLOR_GREEN, -1)     # filtered/good
    curses.init_pair(2, curses.COLOR_YELLOW, -1)    # bias applied / sparkline
    curses.init_pair(3, curses.COLOR_RED, -1)        # calibrating
    curses.init_pair(4, curses.COLOR_CYAN, -1)       # static zeroing
    curses.init_pair(5, curses.COLOR_WHITE, -1)      # normal text
    curses.init_pair(6, curses.COLOR_MAGENTA, -1)    # deadband
    curses.init_pair(7, curses.COLOR_BLACK, curses.COLOR_CYAN)  # edit selection
    # HUD colors
    curses.init_pair(10, curses.COLOR_GREEN, -1)     # HUD border
    curses.init_pair(11, curses.COLOR_GREEN, -1)     # horizon line
    curses.init_pair(12, curses.COLOR_YELLOW, -1)    # pitch ladder
    curses.init_pair(13, curses.COLOR_GREEN, -1)     # FPM wings
    curses.init_pair(14, curses.COLOR_WHITE, -1)     # FPM diamond
    curses.init_pair(15, curses.COLOR_GREEN, -1)     # readouts in HUD
    curses.init_pair(16, curses.COLOR_CYAN, -1)      # yaw tape
    curses.init_pair(17, curses.COLOR_GREEN, -1)     # velocity bar

    SPARK_W = 28

    # Interactive state
    edit_mode = False
    selected_row = 0
    status_msg = ""
    status_ts = 0.0

    while rclpy.ok():
        ch = stdscr.getch()

        if ch == ord("q"):
            break
        elif ch == ord("e"):
            edit_mode = not edit_mode
            status_msg = "EDIT MODE ON" if edit_mode else "EDIT MODE OFF"
            status_ts = time.monotonic()
        elif ch == ord("c"):
            node.recalibrate()
            status_msg = "Recalibrating... keep robot still"
            status_ts = time.monotonic()
        elif ch == ord("s"):
            path = node.save_params()
            status_msg = f"Saved to {path}"
            status_ts = time.monotonic()

        if edit_mode:
            if ch == curses.KEY_UP:
                selected_row = max(0, selected_row - 1)
            elif ch == curses.KEY_DOWN:
                selected_row = min(len(_TUNABLES) - 1, selected_row + 1)
            elif ch in (curses.KEY_RIGHT, ord("+"), ord("=")):
                name, step = _TUNABLES[selected_row]
                _apply_adjustment(node, name, step)
            elif ch in (curses.KEY_LEFT, ord("-")):
                name, step = _TUNABLES[selected_row]
                _apply_adjustment(node, name, -step)
            elif ch in (curses.KEY_SRIGHT, ord("]")):
                name, step = _TUNABLES[selected_row]
                _apply_adjustment(node, name, step * 10)
            elif ch in (curses.KEY_SLEFT, ord("[")):
                name, step = _TUNABLES[selected_row]
                _apply_adjustment(node, name, -step * 10)

        s = node.get_state()
        stdscr.erase()
        h, w = stdscr.getmaxyx()

        def safe(r, c, text, attr=curses.A_NORMAL):
            if r >= h - 1 or c >= w - 1:
                return
            text = text[: w - c - 1]
            try:
                stdscr.addstr(r, c, text, attr)
            except curses.error:
                pass

        # ── Layout: left panel (data) + right panel (HUD) ──
        hud_col = max(50, w - _HUD_W - 2)  # HUD starts here
        data_w = hud_col - 1  # available width for data panel

        row = 0

        # Header
        title = " IMU Filter TUI "
        mode_str = " [EDIT] " if edit_mode else " [AUTO] "
        bar_len = max(0, (w - len(title) - len(mode_str)) // 2)
        bar = "\u2550" * bar_len
        safe(row, 0, f"{bar}{title}{mode_str}{bar}", curses.A_BOLD | curses.color_pair(1))
        row += 1

        # Transient status
        if status_msg and time.monotonic() - status_ts < 3.0:
            safe(row, 0, f" >> {status_msg}", curses.color_pair(2) | curses.A_BOLD)
            row += 1

        # Calibration / filter status
        if s["calibrating"]:
            pct = int(s["cal_progress"] * 100)
            blen = 20
            filled = int(s["cal_progress"] * blen)
            prog = "\u2588" * filled + "\u2591" * (blen - filled)
            safe(row, 0, f" Status: CALIBRATING [{prog}] {pct}%  Keep robot still!",
                 curses.color_pair(3) | curses.A_BOLD)
        else:
            stat_str = "YES" if s["stationary"] else "NO"
            sc = curses.color_pair(4) if s["stationary"] else curses.color_pair(1)
            safe(row, 0, " Status: FILTERING", curses.color_pair(1) | curses.A_BOLD)
            safe(row, 28, f"Stationary: {stat_str}", sc | curses.A_BOLD)
        row += 2

        # ── Tunable Parameters ──
        safe(row, 0, " Tunable Parameters", curses.A_BOLD | curses.A_UNDERLINE)
        if edit_mode:
            safe(row, 22, " [\u2190\u2191\u2192\u2193 adjust, [/] 10x]", curses.color_pair(2))
        row += 1

        tunable_values = [
            s["bias"][0], s["bias"][1], s["bias"][2],
            s["deadband"], s["stationary_vx_thresh"], s["stationary_vyaw_thresh"],
        ]
        tunable_labels = [
            "Bias X", "Bias Y", "Bias Z", "Deadband", "Stat. Vx", "Stat. Vyaw",
        ]
        tunable_fmts = ["{:+.6f}"] * 3 + ["{:.6f}", "{:.4f}", "{:.4f}"]

        for i, (label, val, fmt) in enumerate(zip(tunable_labels, tunable_values, tunable_fmts)):
            marker = " \u25b6 " if (edit_mode and i == selected_row) else "   "
            attr = curses.color_pair(7) | curses.A_BOLD if (edit_mode and i == selected_row) else curses.color_pair(5)
            step_str = f" \u00b1{_TUNABLES[i][1]}" if (edit_mode and i == selected_row) else ""
            safe(row, 0, f"{marker}{label:>10s}: {fmt.format(val)}{step_str}", attr)
            row += 1

        sx, sy, sz = s["stddev"]
        safe(row, 0, f"    \u03c3: x={sx:.6f} y={sy:.6f} z={sz:.6f}",
             curses.color_pair(5) | curses.A_DIM)
        row += 2

        # ── Angular Velocity ──
        safe(row, 0, " Angular Velocity (rad/s)", curses.A_BOLD | curses.A_UNDERLINE)
        row += 1
        safe(row, 0, "         Raw        Filtered    Filter  Sparkline",
             curses.color_pair(5) | curses.A_DIM)
        row += 1

        axis_names = ["Gx", "Gy", "Gz"]
        raw_g = s["raw_gyro"]
        flt_g = s["filt_gyro"]
        flabels = s["filter_labels"]
        raw_hists = [s["hist_raw_gx"], s["hist_raw_gy"], s["hist_raw_gz"]]
        flt_hists = [s["hist_filt_gx"], s["hist_filt_gy"], s["hist_filt_gz"]]

        _FILTER_COLORS = {
            "STATIC": curses.color_pair(4) | curses.A_BOLD,
            "BIAS+DB": curses.color_pair(6) | curses.A_BOLD,
            "BIAS": curses.color_pair(2),
            "CAL": curses.color_pair(3),
        }

        for i in range(3):
            lc = _FILTER_COLORS.get(flabels[i], curses.color_pair(5))
            spark = _sparkline(raw_hists[i], min(SPARK_W, data_w - 48))
            safe(row, 0, f" {axis_names[i]}: {raw_g[i]:+9.6f} \u2192 {flt_g[i]:+9.6f}",
                 curses.color_pair(5))
            safe(row, 35, f"[{flabels[i]:^7s}]", lc)
            safe(row, 45, spark, curses.color_pair(2))
            row += 1

        # Filtered sparklines
        row += 1
        safe(row, 0, " Filtered Gyro", curses.A_BOLD | curses.A_UNDERLINE)
        row += 1
        for i in range(3):
            flt_spark = _sparkline(flt_hists[i], min(SPARK_W, data_w - 7))
            safe(row, 0, f" {axis_names[i]}:", curses.color_pair(5))
            safe(row, 5, flt_spark, curses.color_pair(1))
            row += 1

        row += 1

        # ── Linear Acceleration ──
        safe(row, 0, " Linear Accel (m/s\u00b2)", curses.A_BOLD | curses.A_UNDERLINE)
        row += 1
        accel_names = ["Ax", "Ay", "Az"]
        raw_a = s["raw_accel"]
        ahists = [s["hist_ax"], s["hist_ay"], s["hist_az"]]
        for i in range(3):
            spark = _sparkline(ahists[i], min(SPARK_W, data_w - 48))
            safe(row, 0, f" {accel_names[i]}: {raw_a[i]:+9.4f}", curses.color_pair(5))
            safe(row, 35, f"[ PASS  ]", curses.color_pair(1))
            safe(row, 45, spark, curses.color_pair(5))
            row += 1

        row += 1

        # ── Odometry ──
        safe(row, 0, " Odometry", curses.A_BOLD | curses.A_UNDERLINE)
        row += 1
        sc = curses.color_pair(4) if s["stationary"] else curses.color_pair(5)
        safe(row, 0,
             f" Vx:{s['odom_vx']:+.3f}  Vyaw:{s['odom_vyaw']:+.3f}  "
             f"Stopped:{'YES' if s['stationary'] else 'NO'}",
             sc)
        row += 1
        safe(row, 0,
             f" IMU: {s['imu_rate']:.0f}Hz  N={s['imu_count']}  Pub: /imu/data_filtered",
             curses.color_pair(5) | curses.A_DIM)
        row += 2

        # ── Keybindings ──
        safe(row, 0, " Keys:", curses.A_BOLD)
        row += 1
        binds = "  e:edit  c:recal  s:save  q:quit"
        if edit_mode:
            binds = "  \u2190\u2191\u2192\u2193:tune  []:10x  " + binds
        safe(row, 0, binds, curses.color_pair(5) | curses.A_DIM)

        # ══════════════════════════════════════════════════════
        # RIGHT PANEL: F-16 HUD
        # ══════════════════════════════════════════════════════
        if w >= hud_col + _HUD_W:
            hud_rows, hud_colors, _, _, _, _ = _render_hud(
                s["pitch_deg"], s["roll_deg"],
                s["filt_gyro"][2],  # yaw rate = filtered gyro Z
                s["odom_vx"],
            )

            hud_start_row = 1
            for ri, (txt, cols) in enumerate(zip(hud_rows, hud_colors)):
                tr = hud_start_row + ri
                if tr >= h - 1:
                    break
                # Draw char-by-char for per-character coloring
                for ci, (ch, cp) in enumerate(zip(txt, cols)):
                    col = hud_col + ci
                    if col >= w - 1:
                        break
                    attr = curses.color_pair(cp) if cp else curses.color_pair(10)
                    if cp == 14:  # FPM diamond - make it bold white
                        attr = curses.color_pair(14) | curses.A_BOLD
                    elif cp in (11, 13, 15):  # horizon, FPM wings, readouts
                        attr |= curses.A_BOLD
                    try:
                        stdscr.addstr(tr, col, ch, attr)
                    except curses.error:
                        pass

            # ── Yaw rate tape below HUD ──
            yr = hud_start_row + _HUD_H
            if yr < h - 3:
                yaw_rate = s["filt_gyro"][2]
                safe(yr, hud_col, " YAW RATE", curses.color_pair(16) | curses.A_BOLD)
                yr += 1
                tape_w = _HUD_W - 4
                tape_mid = tape_w // 2
                # Scale: full deflection = 1.0 rad/s
                deflect = max(-1.0, min(1.0, yaw_rate / 1.0))
                needle_pos = int(tape_mid + deflect * tape_mid)
                needle_pos = max(0, min(tape_w - 1, needle_pos))

                tape = list("\u2500" * tape_w)  # ─
                tape[tape_mid] = "\u253c"  # ┼ center mark
                # Quarter marks
                if tape_mid // 2 < tape_w:
                    tape[tape_mid // 2] = "\u2502"
                if tape_mid + tape_mid // 2 < tape_w:
                    tape[tape_mid + tape_mid // 2] = "\u2502"
                tape[needle_pos] = "\u25c6"  # ◆ needle

                safe(yr, hud_col + 2, "".join(tape), curses.color_pair(16))
                safe(yr, hud_col + 2 + tape_w + 1, f"{yaw_rate:+.3f} r/s",
                     curses.color_pair(16) | curses.A_BOLD)
                yr += 1

                # Left/Right labels
                safe(yr, hud_col + 2, "L", curses.color_pair(16) | curses.A_DIM)
                safe(yr, hud_col + 2 + tape_mid, "\u2502", curses.color_pair(16) | curses.A_DIM)
                safe(yr, hud_col + 2 + tape_w - 1, "R", curses.color_pair(16) | curses.A_DIM)
                yr += 1

            # ── Velocity bar below yaw tape ──
            if yr < h - 2:
                vel = s["odom_vx"]
                safe(yr, hud_col, " VELOCITY", curses.color_pair(17) | curses.A_BOLD)
                yr += 1
                bar_w = _HUD_W - 4
                fill = min(abs(vel) / _MAX_VEL_BAR, 1.0)
                filled = int(fill * bar_w)
                arrow = "\u25b6" if vel >= 0 else "\u25c0"
                bar_str = "\u2588" * filled + "\u2591" * (bar_w - filled)
                safe(yr, hud_col + 2, f"{arrow}{bar_str}", curses.color_pair(17))
                safe(yr, hud_col + 2 + bar_w + 2, f"{vel:+.3f} m/s",
                     curses.color_pair(17) | curses.A_BOLD)

        stdscr.refresh()
        time.sleep(0.1)


def _apply_adjustment(node, name, delta):
    """Apply a tuning adjustment to the node."""
    if name == "bias_x":
        node.adjust_bias(0, delta)
    elif name == "bias_y":
        node.adjust_bias(1, delta)
    elif name == "bias_z":
        node.adjust_bias(2, delta)
    elif name == "deadband":
        node.adjust_deadband(delta)
    elif name == "stationary_vx":
        node.adjust_stationary_vx(delta)
    elif name == "stationary_vyaw":
        node.adjust_stationary_vyaw(delta)


def main(args=None):
    rclpy.init(args=args)
    node = ImuFilterTuiNode()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        curses.wrapper(lambda stdscr: _draw_tui(stdscr, node))
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
