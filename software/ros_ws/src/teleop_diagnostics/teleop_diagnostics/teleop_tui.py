#!/usr/bin/env python3
"""
Teleop Diagnostics TUI - A comprehensive diagnostic tool for teleoperation debugging.

Uses curses (standard library) for the terminal interface - no external dependencies.

This tool provides a real-time view of:
- Joystick raw input (axes and buttons)
- Generic controller configuration and scaling
- Velocity command chain (joy -> mux -> controller)
- Wheel states (velocities, positions)
- Twist mux priority and active source
- Wheel axis configuration
"""

import curses
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.executors import MultiThreadedExecutor
import threading
import time
import signal
import sys
from dataclasses import dataclass, field
from typing import Optional, Dict, List
from collections import deque

from sensor_msgs.msg import Joy, JointState
from geometry_msgs.msg import Twist, TwistStamped


@dataclass
class JoyData:
    """Holds joystick data."""
    axes: List[float] = field(default_factory=list)
    buttons: List[int] = field(default_factory=list)
    timestamp: float = 0.0
    rate_hz: float = 0.0


@dataclass
class VelocityData:
    """Holds velocity command data."""
    linear_x: float = 0.0
    angular_z: float = 0.0
    timestamp: float = 0.0
    rate_hz: float = 0.0
    frame_id: str = ""


@dataclass
class WheelData:
    """Holds wheel joint state data."""
    names: List[str] = field(default_factory=list)
    positions: List[float] = field(default_factory=list)
    velocities: List[float] = field(default_factory=list)
    timestamp: float = 0.0
    rate_hz: float = 0.0


@dataclass
class ControllerConfig:
    """Holds generic controller configuration."""
    forward_axis: int = 1
    forward_scaling: float = -0.7
    turn_axis: int = 0
    turn_scaling: float = -0.5
    forward_enable_axis: int = 2
    forward_enable_threshold: float = -0.5
    turbo_enable_axis: int = 5
    turbo_scaling: float = 2.5
    deadband: float = 0.08


class RateCalculator:
    """Calculate message rate from timestamps."""
    def __init__(self, window_size: int = 10):
        self.timestamps: deque = deque(maxlen=window_size)

    def add_timestamp(self, ts: float) -> float:
        self.timestamps.append(ts)
        if len(self.timestamps) < 2:
            return 0.0
        duration = self.timestamps[-1] - self.timestamps[0]
        if duration <= 0:
            return 0.0
        return (len(self.timestamps) - 1) / duration


class TeleopDiagnosticsNode(Node):
    """ROS2 node that collects teleoperation diagnostic data."""

    def __init__(self):
        super().__init__('teleop_diagnostics')

        # Data storage
        self.joy_data = JoyData()
        self.joy_vel_data = VelocityData()
        self.cmd_vel_out_data = VelocityData()
        self.nav_vel_data = VelocityData()
        self.web_vel_data = VelocityData()
        self.key_vel_data = VelocityData()
        self.wheel_data = WheelData()
        self.controller_config = ControllerConfig()

        # Rate calculators
        self.joy_rate = RateCalculator()
        self.joy_vel_rate = RateCalculator()
        self.cmd_vel_out_rate = RateCalculator()
        self.wheel_rate = RateCalculator()

        # Active mux source tracking
        self.active_mux_source = "none"
        self.mux_sources: Dict[str, float] = {
            "joystick": 0.0,
            "keyboard": 0.0,
            "web_ui": 0.0,
            "navigation": 0.0,
        }

        # QoS for sensor data
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        # Subscribe to joy input
        self.joy_sub = self.create_subscription(
            Joy, '/joy', self.joy_callback, sensor_qos)

        # Subscribe to velocity topics
        self.joy_vel_sub = self.create_subscription(
            TwistStamped, '/joy_vel', self.joy_vel_callback, 10)
        self.cmd_vel_out_sub = self.create_subscription(
            TwistStamped, '/cmd_vel_out', self.cmd_vel_out_callback, 10)
        self.nav_vel_sub = self.create_subscription(
            TwistStamped, '/cmd_vel_nav_stamped', self.nav_vel_callback, 10)
        self.web_vel_sub = self.create_subscription(
            TwistStamped, '/web_vel', self.web_vel_callback, 10)
        self.key_vel_sub = self.create_subscription(
            TwistStamped, '/key_vel', self.key_vel_callback, 10)
        self.cmd_vel_unstamped_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_unstamped_callback, 10)

        # Subscribe to joint states
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, sensor_qos)

        self.get_logger().info('Teleop diagnostics node started')

    def joy_callback(self, msg: Joy):
        now = time.time()
        self.joy_data.axes = list(msg.axes)
        self.joy_data.buttons = list(msg.buttons)
        self.joy_data.timestamp = now
        self.joy_data.rate_hz = self.joy_rate.add_timestamp(now)

    def joy_vel_callback(self, msg: TwistStamped):
        now = time.time()
        self.joy_vel_data.linear_x = msg.twist.linear.x
        self.joy_vel_data.angular_z = msg.twist.angular.z
        self.joy_vel_data.timestamp = now
        self.joy_vel_data.frame_id = msg.header.frame_id
        self.joy_vel_data.rate_hz = self.joy_vel_rate.add_timestamp(now)
        self.mux_sources["joystick"] = now
        self._update_active_source()

    def cmd_vel_out_callback(self, msg: TwistStamped):
        now = time.time()
        self.cmd_vel_out_data.linear_x = msg.twist.linear.x
        self.cmd_vel_out_data.angular_z = msg.twist.angular.z
        self.cmd_vel_out_data.timestamp = now
        self.cmd_vel_out_data.frame_id = msg.header.frame_id
        self.cmd_vel_out_data.rate_hz = self.cmd_vel_out_rate.add_timestamp(now)

    def cmd_vel_unstamped_callback(self, msg: Twist):
        now = time.time()
        self.cmd_vel_out_data.linear_x = msg.linear.x
        self.cmd_vel_out_data.angular_z = msg.angular.z
        self.cmd_vel_out_data.timestamp = now
        self.cmd_vel_out_data.rate_hz = self.cmd_vel_out_rate.add_timestamp(now)

    def nav_vel_callback(self, msg: TwistStamped):
        now = time.time()
        self.nav_vel_data.linear_x = msg.twist.linear.x
        self.nav_vel_data.angular_z = msg.twist.angular.z
        self.nav_vel_data.timestamp = now
        self.mux_sources["navigation"] = now
        self._update_active_source()

    def web_vel_callback(self, msg: TwistStamped):
        now = time.time()
        self.web_vel_data.linear_x = msg.twist.linear.x
        self.web_vel_data.angular_z = msg.twist.angular.z
        self.web_vel_data.timestamp = now
        self.mux_sources["web_ui"] = now
        self._update_active_source()

    def key_vel_callback(self, msg: TwistStamped):
        now = time.time()
        self.key_vel_data.linear_x = msg.twist.linear.x
        self.key_vel_data.angular_z = msg.twist.angular.z
        self.key_vel_data.timestamp = now
        self.mux_sources["keyboard"] = now
        self._update_active_source()

    def joint_state_callback(self, msg: JointState):
        now = time.time()
        self.wheel_data.names = list(msg.name)
        self.wheel_data.positions = list(msg.position) if msg.position else []
        self.wheel_data.velocities = list(msg.velocity) if msg.velocity else []
        self.wheel_data.timestamp = now
        self.wheel_data.rate_hz = self.wheel_rate.add_timestamp(now)

    def _update_active_source(self):
        """Determine which mux source is currently active."""
        now = time.time()
        timeout = 0.5
        priority_order = ["joystick", "keyboard", "web_ui", "navigation"]
        for source in priority_order:
            if now - self.mux_sources.get(source, 0.0) < timeout:
                self.active_mux_source = source
                return
        self.active_mux_source = "none"


class TeleopTUI:
    """Curses-based TUI for teleop diagnostics."""

    MUX_PRIORITIES = {
        "joystick": 100,
        "keyboard": 90,
        "web_ui": 80,
        "navigation": 10,
    }

    def __init__(self, node: TeleopDiagnosticsNode):
        self.node = node
        self.running = True
        self.stdscr = None

    def make_bar(self, value: float, width: int = 20) -> str:
        """Create a text bar for value in range [-1, 1]."""
        normalized = (value + 1) / 2  # Convert to 0-1
        normalized = max(0, min(1, normalized))
        mid = width // 2
        pos = int(normalized * width)
        bar = ['-'] * width
        bar[mid] = '|'
        if pos < width:
            bar[pos] = '*'
        return '[' + ''.join(bar) + ']'

    def draw_box(self, y: int, x: int, h: int, w: int, title: str = ""):
        """Draw a box with optional title."""
        # Top border
        self.stdscr.addstr(y, x, '+' + '-' * (w - 2) + '+')
        # Title
        if title:
            title_str = f" {title} "
            self.stdscr.addstr(y, x + 2, title_str, curses.A_BOLD)
        # Sides
        for i in range(1, h - 1):
            self.stdscr.addstr(y + i, x, '|')
            self.stdscr.addstr(y + i, x + w - 1, '|')
        # Bottom border
        self.stdscr.addstr(y + h - 1, x, '+' + '-' * (w - 2) + '+')

    def safe_addstr(self, y: int, x: int, text: str, attr=0):
        """Safely add string, handling screen boundaries."""
        max_y, max_x = self.stdscr.getmaxyx()
        if y < 0 or y >= max_y or x < 0:
            return
        # Truncate if needed
        available = max_x - x - 1
        if available <= 0:
            return
        self.stdscr.addstr(y, x, text[:available], attr)

    def draw_joy_panel(self, y: int, x: int, w: int):
        """Draw joystick input panel."""
        h = 14
        self.draw_box(y, x, h, w, "Joy Input (/joy)")

        joy = self.node.joy_data
        now = time.time()
        stale = now - joy.timestamp > 1.0

        # Rate display
        rate_str = f"Rate: {joy.rate_hz:.1f} Hz"
        if stale:
            rate_str += " [STALE]"
        self.safe_addstr(y + 1, x + 2, rate_str)

        # Axes
        self.safe_addstr(y + 2, x + 2, "Axes:", curses.A_BOLD)
        for i, val in enumerate(joy.axes[:8]):
            bar = self.make_bar(val, 16)
            line = f"[{i}] {val:+.2f} {bar}"
            attr = curses.A_BOLD if abs(val) > 0.1 else curses.A_DIM
            self.safe_addstr(y + 3 + i, x + 2, line, attr)

        # Buttons
        self.safe_addstr(y + 11, x + 2, "Buttons:", curses.A_BOLD)
        btn_str = ' '.join([str(i) if b else '.' for i, b in enumerate(joy.buttons[:12])])
        self.safe_addstr(y + 12, x + 2, btn_str)

    def draw_config_panel(self, y: int, x: int, w: int):
        """Draw controller config panel."""
        h = 12
        self.draw_box(y, x, h, w, "Controller Config")

        cfg = self.node.controller_config
        lines = [
            f"Forward Axis:   {cfg.forward_axis}  (Left stick Y)",
            f"Forward Scale:  {cfg.forward_scaling:+.2f}",
            f"Turn Axis:      {cfg.turn_axis}  (Left stick X)",
            f"Turn Scale:     {cfg.turn_scaling:+.2f}",
            "",
            f"Enable Axis:    {cfg.forward_enable_axis}  (RT)",
            f"Enable Thresh:  {cfg.forward_enable_threshold:+.2f}",
            f"Turbo Axis:     {cfg.turbo_enable_axis}  (LT)",
            f"Turbo Scale:    {cfg.turbo_scaling:.1f}x",
        ]
        for i, line in enumerate(lines):
            self.safe_addstr(y + 1 + i, x + 2, line)

    def draw_velocity_panel(self, y: int, x: int, w: int):
        """Draw velocity command chain panel."""
        h = 14
        self.draw_box(y, x, h, w, "Velocity Command Chain")

        now = time.time()
        joy = self.node.joy_vel_data
        out = self.node.cmd_vel_out_data
        nav = self.node.nav_vel_data

        def dir_str(val: float) -> str:
            if val > 0.05:
                return "FWD >>>"
            elif val < -0.05:
                return "<<< REV"
            return "STOPPED"

        def rot_str(val: float) -> str:
            if val > 0.05:
                return "<<< LEFT"
            elif val < -0.05:
                return "RIGHT >>>"
            return "STRAIGHT"

        # Header
        self.safe_addstr(y + 1, x + 2, f"{'Source':<12} {'Lin.X':>8} {'Ang.Z':>8} {'Direction':<12} {'Status':<8}", curses.A_BOLD)

        # Joy vel
        joy_stale = now - joy.timestamp > 0.5
        joy_status = "STALE" if joy_stale else "OK"
        self.safe_addstr(y + 3, x + 2, f"{'joy_vel':<12} {joy.linear_x:+7.3f} {joy.angular_z:+7.3f}  {dir_str(joy.linear_x):<12} {joy_status}")

        # Nav vel (if active)
        nav_active = now - nav.timestamp < 0.5
        if nav_active:
            self.safe_addstr(y + 4, x + 2, f"{'nav_vel':<12} {nav.linear_x:+7.3f} {nav.angular_z:+7.3f}  {dir_str(nav.linear_x):<12} OK")

        # Separator
        self.safe_addstr(y + 5, x + 2, "-" * (w - 4) + " TWIST MUX")

        # Output
        out_stale = now - out.timestamp > 0.5
        out_status = "STALE" if out_stale else "OK"
        self.safe_addstr(y + 7, x + 2, f"{'cmd_vel_out':<12} {out.linear_x:+7.3f} {out.angular_z:+7.3f}  {dir_str(out.linear_x):<12} {out_status}",
                        curses.A_BOLD)

        # Expected wheel velocities
        wheel_sep = 0.714
        wheel_rad = 0.147
        left_vel = (out.linear_x - out.angular_z * wheel_sep / 2) / wheel_rad
        right_vel = (out.linear_x + out.angular_z * wheel_sep / 2) / wheel_rad

        self.safe_addstr(y + 9, x + 2, "-" * (w - 4) + " DIFF DRIVE")
        self.safe_addstr(y + 10, x + 2, f"Expected wheels:  L: {left_vel:+.2f} rad/s   R: {right_vel:+.2f} rad/s")

        # Direction summary
        self.safe_addstr(y + 12, x + 2, f"Motion: {dir_str(out.linear_x)}  |  Turn: {rot_str(out.angular_z)}", curses.A_BOLD)

    def draw_wheel_panel(self, y: int, x: int, w: int):
        """Draw wheel state panel."""
        h = 10
        self.draw_box(y, x, h, w, "Wheel States")

        wheels = self.node.wheel_data
        now = time.time()
        stale = now - wheels.timestamp > 1.0

        rate_str = f"Rate: {wheels.rate_hz:.1f} Hz"
        if stale:
            rate_str += " [STALE]"
        self.safe_addstr(y + 1, x + 2, rate_str)

        self.safe_addstr(y + 2, x + 2, f"{'Joint':<28} {'Vel':>8} {'Dir':<6}", curses.A_BOLD)

        row = 3
        for i, name in enumerate(wheels.names):
            if 'wheel' in name.lower():
                vel = wheels.velocities[i] if i < len(wheels.velocities) else 0.0
                if vel > 0.01:
                    direction = "FWD"
                elif vel < -0.01:
                    direction = "REV"
                else:
                    direction = "---"
                self.safe_addstr(y + row, x + 2, f"{name:<28} {vel:+7.3f}  {direction}")
                row += 1
                if row >= h - 2:
                    break

        # Axis config
        self.safe_addstr(y + h - 2, x + 2, "Axis: xyz=\"0 -1 0\" (+vel=FWD)", curses.A_DIM)

    def draw_mux_panel(self, y: int, x: int, w: int):
        """Draw twist mux status panel."""
        h = 8
        self.draw_box(y, x, h, w, "Twist Mux")

        now = time.time()
        topics = {
            "joystick": "joy_vel",
            "keyboard": "key_vel",
            "web_ui": "web_vel",
            "navigation": "cmd_vel_nav_stamped",
        }

        self.safe_addstr(y + 1, x + 2, f"{'Source':<12} {'Pri':>4} {'Topic':<20} {'Status':<8}", curses.A_BOLD)

        row = 2
        for source in ["joystick", "keyboard", "web_ui", "navigation"]:
            priority = self.MUX_PRIORITIES[source]
            topic = topics[source]
            last_time = self.node.mux_sources.get(source, 0.0)
            active = now - last_time < 0.5
            is_current = source == self.node.active_mux_source

            if is_current and active:
                status = "ACTIVE"
                attr = curses.A_BOLD | curses.A_REVERSE
            elif active:
                status = "ready"
                attr = curses.A_BOLD
            else:
                status = "---"
                attr = curses.A_DIM

            self.safe_addstr(y + row, x + 2, f"{source:<12} {priority:>4} {topic:<20} {status:<8}", attr)
            row += 1

    def draw_direction_panel(self, y: int, x: int, w: int):
        """Draw direction indicator panel."""
        h = 10
        self.draw_box(y, x, h, w, "Direction")

        out = self.node.cmd_vel_out_data
        lin = out.linear_x
        ang = out.angular_z

        # ASCII robot
        if lin > 0.05:
            self.safe_addstr(y + 2, x + w//2 - 1, "^", curses.A_BOLD)
            self.safe_addstr(y + 3, x + w//2 - 1, "|", curses.A_BOLD)
            self.safe_addstr(y + 4, x + w//2 - 1, "|", curses.A_BOLD)
        elif lin < -0.05:
            self.safe_addstr(y + 2, x + w//2 - 1, "|", curses.A_BOLD)
            self.safe_addstr(y + 3, x + w//2 - 1, "|", curses.A_BOLD)
            self.safe_addstr(y + 4, x + w//2 - 1, "v", curses.A_BOLD)
        else:
            self.safe_addstr(y + 3, x + w//2 - 1, "o", curses.A_DIM)

        # Rotation
        if ang > 0.05:
            rot = "<-- LEFT"
        elif ang < -0.05:
            rot = "RIGHT -->"
        else:
            rot = "STRAIGHT"
        self.safe_addstr(y + 6, x + 2, rot, curses.A_BOLD)

        # Speed
        speed = abs(lin)
        if speed > 1.0:
            speed_txt = "FAST"
        elif speed > 0.3:
            speed_txt = "MEDIUM"
        elif speed > 0.05:
            speed_txt = "SLOW"
        else:
            speed_txt = "STOPPED"
        self.safe_addstr(y + 8, x + 2, f"Speed: {speed_txt} ({speed:.2f} m/s)")

    def run_curses(self, stdscr):
        """Main curses loop."""
        self.stdscr = stdscr
        curses.curs_set(0)  # Hide cursor
        stdscr.nodelay(True)  # Non-blocking input
        stdscr.timeout(100)  # 100ms refresh

        while self.running:
            try:
                stdscr.clear()
                max_y, max_x = stdscr.getmaxyx()

                # Title bar
                title = " TELEOP DIAGNOSTICS TUI - Press 'q' to quit "
                self.safe_addstr(0, 0, title.center(max_x), curses.A_REVERSE | curses.A_BOLD)

                # Calculate column widths
                col1_w = 40
                col2_w = max(50, max_x - col1_w - 30)
                col3_w = 28

                # Left column: Joy + Config
                self.draw_joy_panel(2, 0, col1_w)
                self.draw_config_panel(16, 0, col1_w)

                # Center column: Velocity + Wheels
                self.draw_velocity_panel(2, col1_w + 1, col2_w)
                self.draw_wheel_panel(17, col1_w + 1, col2_w)

                # Right column: Mux + Direction
                self.draw_mux_panel(2, col1_w + col2_w + 2, col3_w)
                self.draw_direction_panel(11, col1_w + col2_w + 2, col3_w)

                # Status bar
                status = f" Time: {time.strftime('%H:%M:%S')} | Joy: {self.node.joy_data.rate_hz:.1f}Hz | Cmd: {self.node.cmd_vel_out_data.rate_hz:.1f}Hz | Wheels: {self.node.wheel_data.rate_hz:.1f}Hz "
                self.safe_addstr(max_y - 1, 0, status.ljust(max_x), curses.A_REVERSE)

                stdscr.refresh()

                # Check for quit
                key = stdscr.getch()
                if key == ord('q') or key == ord('Q'):
                    self.running = False

            except curses.error:
                pass
            except KeyboardInterrupt:
                self.running = False

    def run(self):
        """Run the TUI."""
        curses.wrapper(self.run_curses)

    def stop(self):
        """Stop the TUI."""
        self.running = False


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)

    node = TeleopDiagnosticsNode()
    tui = TeleopTUI(node)

    # Handle SIGINT
    def signal_handler(sig, frame):
        tui.stop()

    signal.signal(signal.SIGINT, signal_handler)

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
