#!/usr/bin/env python3
"""
Teleop Diagnostics TUI - A comprehensive diagnostic tool for teleoperation debugging.

This tool provides a real-time view of:
- Joystick raw input (axes and buttons)
- Generic controller configuration and scaling
- Velocity command chain (joy -> mux -> controller)
- Wheel states (velocities, positions)
- Twist mux priority and active source
- Wheel axis configuration
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.executors import MultiThreadedExecutor
import threading
import time
import signal
import sys
from dataclasses import dataclass, field
from typing import Optional, Dict, List, Any
from collections import deque

from sensor_msgs.msg import Joy, JointState
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import String

try:
    from rich.console import Console
    from rich.live import Live
    from rich.table import Table
    from rich.panel import Panel
    from rich.layout import Layout
    from rich.text import Text
    from rich import box
    RICH_AVAILABLE = True
except ImportError:
    RICH_AVAILABLE = False
    print("Warning: 'rich' library not installed. Install with: pip install rich")
    print("Falling back to simple text output.")


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
    forward_axis: int = -1
    forward_scaling: float = 0.0
    turn_axis: int = -1
    turn_scaling: float = 0.0
    forward_enable_axis: int = -1
    forward_enable_threshold: float = 0.0
    turbo_enable_axis: int = -1
    turbo_scaling: float = 0.0
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

        # Subscribe to velocity topics (all potential mux inputs)
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

        # Also try unstamped versions
        self.cmd_vel_unstamped_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_unstamped_callback, 10)

        # Subscribe to joint states for wheel data
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, sensor_qos)

        # Try to get controller parameters
        self._fetch_controller_params()

        self.get_logger().info('Teleop diagnostics node started')

    def _fetch_controller_params(self):
        """Attempt to fetch generic_controller parameters."""
        # This would typically use a service call to get parameters
        # For now, we'll use default values from the config
        self.controller_config.forward_axis = 1
        self.controller_config.forward_scaling = -0.7
        self.controller_config.turn_axis = 0
        self.controller_config.turn_scaling = -0.5
        self.controller_config.forward_enable_axis = 2
        self.controller_config.forward_enable_threshold = -0.5
        self.controller_config.turbo_enable_axis = 5
        self.controller_config.turbo_scaling = 2.5

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
        """Determine which mux source is currently active (most recent with non-zero velocity)."""
        now = time.time()
        timeout = 0.5  # Match mux timeout

        # Priority order (highest first): joystick(100), keyboard(90), web_ui(80), navigation(10)
        priority_order = ["joystick", "keyboard", "web_ui", "navigation"]

        for source in priority_order:
            last_time = self.mux_sources.get(source, 0.0)
            if now - last_time < timeout:
                self.active_mux_source = source
                return

        self.active_mux_source = "none"


class TeleopTUI:
    """Text User Interface for teleop diagnostics."""

    # Wheel axis configuration (from URDF)
    WHEEL_AXIS = "Y-axis: xyz=\"0 -1 0\""
    WHEEL_AXIS_COMMENT = "Positive velocity = Forward (REP-103)"

    # Mux priorities (from twist_mux.yaml)
    MUX_PRIORITIES = {
        "joystick": 100,
        "keyboard": 90,
        "web_ui": 80,
        "navigation": 10,
    }

    def __init__(self, node: TeleopDiagnosticsNode):
        self.node = node
        self.console = Console()
        self.running = True

    def create_joy_panel(self) -> Panel:
        """Create panel showing raw joystick input."""
        table = Table(box=box.SIMPLE, show_header=True, header_style="bold cyan")
        table.add_column("Axis", justify="right", width=6)
        table.add_column("Value", justify="center", width=8)
        table.add_column("Bar", justify="left", width=20)

        axes = self.node.joy_data.axes or []
        for i, val in enumerate(axes[:8]):  # Show first 8 axes
            bar = self._make_bar(val, -1.0, 1.0, 20)
            style = "green" if abs(val) > 0.1 else "dim"
            table.add_row(f"[{i}]", f"{val:+.2f}", bar, style=style)

        # Buttons row
        buttons = self.node.joy_data.buttons or []
        btn_str = " ".join([f"[{'green' if b else 'dim'}]{i}[/]" for i, b in enumerate(buttons[:12])])

        rate = self.node.joy_data.rate_hz
        stale = time.time() - self.node.joy_data.timestamp > 1.0
        status = f"[red]STALE[/]" if stale else f"[green]{rate:.1f} Hz[/]"

        content = Text()
        content.append_text(Text.from_markup(f"Rate: {status}\n"))
        content.append_text(Text.from_markup(f"Buttons: {btn_str}\n\n"))

        return Panel(
            table,
            title="[bold]Joy Input (/joy)[/]",
            subtitle=f"Rate: {status}",
            border_style="blue"
        )

    def create_controller_config_panel(self) -> Panel:
        """Create panel showing controller configuration."""
        cfg = self.node.controller_config

        table = Table(box=box.SIMPLE, show_header=False)
        table.add_column("Parameter", style="cyan", width=20)
        table.add_column("Value", width=15)
        table.add_column("Note", style="dim", width=25)

        # Forward/Turn config
        table.add_row("Forward Axis", f"{cfg.forward_axis}", "Left stick Y")
        table.add_row("Forward Scale", f"{cfg.forward_scaling:+.2f}",
                     "[yellow]Negative[/] = invert" if cfg.forward_scaling < 0 else "")
        table.add_row("Turn Axis", f"{cfg.turn_axis}", "Left stick X")
        table.add_row("Turn Scale", f"{cfg.turn_scaling:+.2f}",
                     "[yellow]Negative[/] = invert" if cfg.turn_scaling < 0 else "")
        table.add_row("", "", "")
        table.add_row("Enable Axis", f"{cfg.forward_enable_axis}", "Right trigger")
        table.add_row("Enable Thresh", f"{cfg.forward_enable_threshold:+.2f}", "< threshold = enabled")
        table.add_row("Turbo Axis", f"{cfg.turbo_enable_axis}", "Left trigger")
        table.add_row("Turbo Scale", f"{cfg.turbo_scaling:.2f}x", "")
        table.add_row("Deadband", f"{cfg.deadband:.2f}", "")

        return Panel(
            table,
            title="[bold]Controller Config[/]",
            subtitle="generic_controller params",
            border_style="magenta"
        )

    def create_velocity_chain_panel(self) -> Panel:
        """Create panel showing velocity command chain."""
        joy = self.node.joy_vel_data
        out = self.node.cmd_vel_out_data

        # Calculate if commands are being processed
        now = time.time()
        joy_active = now - joy.timestamp < 0.5
        out_active = now - out.timestamp < 0.5

        # Direction indicators
        def dir_indicator(val: float) -> str:
            if val > 0.05:
                return "[green]FWD >>>[/]"
            elif val < -0.05:
                return "[red]<<< REV[/]"
            else:
                return "[dim]STOP[/]"

        def rot_indicator(val: float) -> str:
            if val > 0.05:
                return "[cyan]<<< LEFT[/]"
            elif val < -0.05:
                return "[yellow]RIGHT >>>[/]"
            else:
                return "[dim]STRAIGHT[/]"

        table = Table(box=box.ROUNDED, show_header=True, header_style="bold")
        table.add_column("Stage", width=18)
        table.add_column("Linear X", justify="center", width=10)
        table.add_column("Angular Z", justify="center", width=10)
        table.add_column("Direction", justify="center", width=14)
        table.add_column("Status", justify="center", width=10)

        # Joy velocity (from generic_controller)
        joy_status = "[green]OK[/]" if joy_active else "[red]STALE[/]"
        table.add_row(
            "[bold cyan]joy_vel[/]",
            f"{joy.linear_x:+.3f}",
            f"{joy.angular_z:+.3f}",
            dir_indicator(joy.linear_x),
            joy_status
        )

        # Nav velocity
        nav = self.node.nav_vel_data
        nav_active = now - nav.timestamp < 0.5
        if nav_active:
            table.add_row(
                "[bold blue]nav_vel[/]",
                f"{nav.linear_x:+.3f}",
                f"{nav.angular_z:+.3f}",
                dir_indicator(nav.linear_x),
                "[green]OK[/]"
            )

        # Separator
        table.add_row("[dim]--- TWIST MUX ---[/]", "", "", "", "")

        # Output velocity
        out_status = "[green]OK[/]" if out_active else "[red]STALE[/]"
        table.add_row(
            "[bold green]cmd_vel_out[/]",
            f"{out.linear_x:+.3f}",
            f"{out.angular_z:+.3f}",
            dir_indicator(out.linear_x),
            out_status
        )

        # Compute expected wheel velocities (simplified diff drive kinematics)
        wheel_sep = 0.714  # meters
        wheel_rad = 0.147  # meters
        left_vel = (out.linear_x - out.angular_z * wheel_sep / 2) / wheel_rad
        right_vel = (out.linear_x + out.angular_z * wheel_sep / 2) / wheel_rad

        table.add_row("[dim]--- DIFF DRIVE ---[/]", "", "", "", "")
        table.add_row(
            "[yellow]Expected L/R[/]",
            f"L:{left_vel:+.2f}",
            f"R:{right_vel:+.2f}",
            "[dim]rad/s[/]",
            ""
        )

        return Panel(
            table,
            title="[bold]Velocity Command Chain[/]",
            border_style="green"
        )

    def create_wheel_state_panel(self) -> Panel:
        """Create panel showing wheel states."""
        wheels = self.node.wheel_data

        table = Table(box=box.SIMPLE, show_header=True, header_style="bold")
        table.add_column("Joint", width=25)
        table.add_column("Position", justify="center", width=10)
        table.add_column("Velocity", justify="center", width=10)
        table.add_column("Dir", justify="center", width=8)

        # Filter for wheel joints
        wheel_keywords = ["wheel", "front", "rear"]
        for i, name in enumerate(wheels.names):
            if any(kw in name.lower() for kw in wheel_keywords):
                pos = wheels.positions[i] if i < len(wheels.positions) else 0.0
                vel = wheels.velocities[i] if i < len(wheels.velocities) else 0.0

                # Direction indicator
                if vel > 0.01:
                    direction = "[green]FWD[/]"
                elif vel < -0.01:
                    direction = "[red]REV[/]"
                else:
                    direction = "[dim]---[/]"

                table.add_row(name, f"{pos:.3f}", f"{vel:+.3f}", direction)

        if not wheels.names:
            table.add_row("[dim]No wheel data[/]", "", "", "")

        rate = wheels.rate_hz
        stale = time.time() - wheels.timestamp > 1.0
        status = f"[red]STALE[/]" if stale else f"[green]{rate:.1f} Hz[/]"

        # Add axis configuration info
        axis_info = Text()
        axis_info.append(f"\nWheel Axis Config: ", style="bold")
        axis_info.append(self.WHEEL_AXIS, style="yellow")
        axis_info.append(f"\n{self.WHEEL_AXIS_COMMENT}", style="dim")

        return Panel(
            Text.assemble(table, axis_info),
            title="[bold]Wheel Joint States[/]",
            subtitle=f"Rate: {status}",
            border_style="yellow"
        )

    def create_mux_panel(self) -> Panel:
        """Create panel showing twist mux status."""
        table = Table(box=box.SIMPLE, show_header=True, header_style="bold")
        table.add_column("Source", width=12)
        table.add_column("Priority", justify="center", width=8)
        table.add_column("Topic", width=20)
        table.add_column("Status", justify="center", width=10)

        now = time.time()
        source_topics = {
            "joystick": "joy_vel",
            "keyboard": "key_vel",
            "web_ui": "web_vel",
            "navigation": "cmd_vel_nav_stamped",
        }

        for source in ["joystick", "keyboard", "web_ui", "navigation"]:
            priority = self.MUX_PRIORITIES[source]
            topic = source_topics[source]
            last_time = self.node.mux_sources.get(source, 0.0)
            active = now - last_time < 0.5

            is_current = source == self.node.active_mux_source

            if is_current and active:
                status = "[bold green]ACTIVE[/]"
                style = "bold green"
            elif active:
                status = "[yellow]ready[/]"
                style = "yellow"
            else:
                status = "[dim]inactive[/]"
                style = "dim"

            name = f"[{style}]{source}[/]"
            table.add_row(name, str(priority), topic, status)

        return Panel(
            table,
            title="[bold]Twist Mux Status[/]",
            subtitle=f"Active: {self.node.active_mux_source}",
            border_style="cyan"
        )

    def create_direction_summary(self) -> Panel:
        """Create a visual direction summary."""
        out = self.node.cmd_vel_out_data

        # Create ASCII art robot direction indicator
        lin = out.linear_x
        ang = out.angular_z

        lines = []

        # Forward/backward arrow
        if lin > 0.05:
            lines.append("       [bold green]^[/]")
            lines.append("       [bold green]|[/]")
            lines.append("       [bold green]|[/]")
        elif lin < -0.05:
            lines.append("       [bold red]|[/]")
            lines.append("       [bold red]|[/]")
            lines.append("       [bold red]v[/]")
        else:
            lines.append("        ")
            lines.append("       [dim]o[/]")
            lines.append("        ")

        # Add rotation indicator
        if ang > 0.05:
            rot = "[cyan]<<< TURNING LEFT[/]"
        elif ang < -0.05:
            rot = "[yellow]TURNING RIGHT >>>[/]"
        else:
            rot = "[dim]STRAIGHT[/]"

        # Speed magnitude
        speed = abs(lin)
        if speed > 1.0:
            speed_txt = "[bold red]FAST[/]"
        elif speed > 0.3:
            speed_txt = "[yellow]MEDIUM[/]"
        elif speed > 0.05:
            speed_txt = "[green]SLOW[/]"
        else:
            speed_txt = "[dim]STOPPED[/]"

        content = Text()
        for line in lines:
            content.append_text(Text.from_markup(line + "\n"))
        content.append("\n")
        content.append_text(Text.from_markup(f"   {rot}\n"))
        content.append_text(Text.from_markup(f"   Speed: {speed_txt} ({speed:.2f} m/s)\n"))

        return Panel(
            content,
            title="[bold]Robot Direction[/]",
            border_style="white"
        )

    def _make_bar(self, value: float, min_val: float, max_val: float, width: int) -> str:
        """Create a text-based bar visualization."""
        normalized = (value - min_val) / (max_val - min_val)
        normalized = max(0, min(1, normalized))

        mid = width // 2
        pos = int(normalized * width)

        bar = [" "] * width
        bar[mid] = "|"

        if pos < mid:
            for i in range(pos, mid):
                bar[i] = "="
        elif pos > mid:
            for i in range(mid + 1, pos + 1):
                bar[i] = "="

        bar[pos] = "*"

        return "[" + "".join(bar) + "]"

    def create_layout(self) -> Layout:
        """Create the main layout."""
        layout = Layout()

        layout.split_column(
            Layout(name="top", size=3),
            Layout(name="main"),
            Layout(name="bottom", size=3)
        )

        layout["top"].update(Panel(
            "[bold]Teleop Diagnostics TUI[/] - Press Ctrl+C to exit",
            style="bold white on blue"
        ))

        layout["main"].split_row(
            Layout(name="left", ratio=1),
            Layout(name="center", ratio=2),
            Layout(name="right", ratio=1)
        )

        layout["left"].split_column(
            Layout(name="joy", ratio=2),
            Layout(name="config", ratio=1)
        )

        layout["center"].split_column(
            Layout(name="velocity", ratio=2),
            Layout(name="wheels", ratio=1)
        )

        layout["right"].split_column(
            Layout(name="mux", ratio=1),
            Layout(name="direction", ratio=1)
        )

        layout["bottom"].update(Panel(
            f"[dim]Time: {time.strftime('%H:%M:%S')} | "
            f"Joy: {self.node.joy_data.rate_hz:.1f}Hz | "
            f"Wheels: {self.node.wheel_data.rate_hz:.1f}Hz[/]",
            style="dim"
        ))

        return layout

    def update_layout(self, layout: Layout):
        """Update all panels in the layout."""
        layout["joy"].update(self.create_joy_panel())
        layout["config"].update(self.create_controller_config_panel())
        layout["velocity"].update(self.create_velocity_chain_panel())
        layout["wheels"].update(self.create_wheel_state_panel())
        layout["mux"].update(self.create_mux_panel())
        layout["direction"].update(self.create_direction_summary())

        layout["bottom"].update(Panel(
            f"[dim]Time: {time.strftime('%H:%M:%S')} | "
            f"Joy: {self.node.joy_data.rate_hz:.1f}Hz | "
            f"Cmd: {self.node.cmd_vel_out_data.rate_hz:.1f}Hz | "
            f"Wheels: {self.node.wheel_data.rate_hz:.1f}Hz[/]",
            style="dim"
        ))

    def run(self):
        """Run the TUI."""
        if not RICH_AVAILABLE:
            self.run_simple()
            return

        layout = self.create_layout()

        with Live(layout, console=self.console, refresh_per_second=10, screen=True) as live:
            while self.running:
                try:
                    self.update_layout(layout)
                    time.sleep(0.1)
                except KeyboardInterrupt:
                    self.running = False
                    break

    def run_simple(self):
        """Run simple text output mode (fallback when rich is not available)."""
        while self.running:
            try:
                print("\033[2J\033[H")  # Clear screen
                print("=" * 60)
                print("TELEOP DIAGNOSTICS (simple mode)")
                print("=" * 60)

                joy = self.node.joy_data
                print(f"\nJoy Axes: {[f'{a:.2f}' for a in joy.axes[:6]]}")
                print(f"Joy Buttons: {joy.buttons[:8]}")
                print(f"Joy Rate: {joy.rate_hz:.1f} Hz")

                vel = self.node.cmd_vel_out_data
                print(f"\nCmd Vel Out: linear={vel.linear_x:.3f}, angular={vel.angular_z:.3f}")

                wheels = self.node.wheel_data
                print(f"\nWheel Names: {wheels.names}")
                print(f"Velocities: {[f'{v:.3f}' for v in wheels.velocities]}")

                print(f"\nActive Mux Source: {self.node.active_mux_source}")
                print("\nPress Ctrl+C to exit")

                time.sleep(0.5)
            except KeyboardInterrupt:
                self.running = False
                break

    def stop(self):
        """Stop the TUI."""
        self.running = False


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)

    node = TeleopDiagnosticsNode()
    tui = TeleopTUI(node)

    # Handle SIGINT gracefully
    def signal_handler(sig, frame):
        tui.stop()
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)

    # Run ROS spinner in background thread
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
