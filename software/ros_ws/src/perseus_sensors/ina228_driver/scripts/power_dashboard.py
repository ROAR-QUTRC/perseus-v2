#!/usr/bin/env python3
"""Terminal dashboard with braille sparkline graphs for INA228 power monitor."""

import signal
import sys
import time
from collections import deque

import rclpy
from rclpy.node import Node
from perseus_interfaces.msg import DCPowerData

HISTORY = 120
GRAPH_CHARS = 20  # 20 braille chars = 40 data points displayed

# Braille dot layout per character (2 columns x 4 rows):
#   dot1 dot4      bit0 bit3
#   dot2 dot5      bit1 bit4
#   dot3 dot6      bit2 bit5
#   dot7 dot8      bit6 bit7
# Fill from bottom to top for each column.
_L = [0x00, 0x40, 0x44, 0x46, 0x47]  # left column, 0-4 dots
_R = [0x00, 0x80, 0xA0, 0xB0, 0xB8]  # right column, 0-4 dots


def sparkline(values, width=GRAPH_CHARS, color="32"):
    """Render a braille sparkline from a deque of floats."""
    data = list(values)[-(width * 2) :]
    if not data:
        return f"\033[90m{chr(0x2800) * width}\033[0m"

    # Pad left if not enough data yet
    data = [None] * (width * 2 - len(data)) + data

    valid = [v for v in data if v is not None]
    if not valid:
        return f"\033[90m{chr(0x2800) * width}\033[0m"

    lo, hi = min(valid), max(valid)

    def lvl(v):
        if v is None:
            return 0
        if hi == lo:
            return 2
        return min(4, max(1, int((v - lo) / (hi - lo) * 4) + 1))

    chars = []
    for i in range(0, len(data), 2):
        left = lvl(data[i])
        right = lvl(data[i + 1]) if i + 1 < len(data) else 0
        chars.append(chr(0x2800 | _L[left] | _R[right]))

    return f"\033[{color}m{''.join(chars)}\033[0m"


# (label, msg attribute, display unit, format string, scale factor, graph color)
FIELDS = [
    ("Bus Voltage", "bus_voltage", "V", "{:10.4f}", 1.0, "93"),
    ("Shunt Voltage", "shunt_voltage", "mV", "{:10.4f}", 1e3, "94"),
    ("Current", "current", "A", "{:10.4f}", 1.0, "92"),
    ("Power", "power", "W", "{:10.4f}", 1.0, "91"),
    ("Energy", "energy", "J", "{:10.4f}", 1.0, "95"),
    ("Charge", "charge", "C", "{:10.4f}", 1.0, "96"),
    ("Die Temp", "die_temperature", "\u00b0C", "{:10.2f}", 1.0, "33"),
]

# Box inner width: 2 + 15 + 10 + 1 + 3 + 1 + GRAPH_CHARS + padding
W = 58


class PowerDashboard(Node):
    def __init__(self):
        super().__init__("power_dashboard")
        self._sub = self.create_subscription(
            DCPowerData, "power_monitor/data", self._callback, 10
        )
        self._msg = None
        self._count = 0
        self._start = time.monotonic()
        self._last_time = None
        self._history = {f[1]: deque(maxlen=HISTORY) for f in FIELDS}
        self.create_timer(0.25, self._draw)
        sys.stdout.write("\033[?25l\033[2J")
        sys.stdout.flush()

    def _callback(self, msg):
        self._msg = msg
        self._count += 1
        self._last_time = time.monotonic()
        for _, attr, _, _, scale, _ in FIELDS:
            self._history[attr].append(getattr(msg, attr) * scale)

    def _draw(self):
        elapsed = time.monotonic() - self._start
        rate = self._count / elapsed if elapsed > 0 else 0.0

        bar = "\u2550" * W
        out = "\033[H"
        out += f"\033[1;36m\u2554{bar}\u2557\033[0m\n"
        title = "INA228 DC Power Monitor"
        out += f"\033[1;36m\u2551\033[0m   \033[1;37m{title}\033[0m{' ' * (W - len(title) - 3)}\033[1;36m\u2551\033[0m\n"
        out += f"\033[1;36m\u2560{bar}\u2563\033[0m\n"

        if self._msg is None:
            out += f"\033[1;36m\u2551\033[0m{' ' * W}\033[1;36m\u2551\033[0m\n"
            wait = "Waiting for data..."
            out += f"\033[1;36m\u2551\033[0m   \033[33m{wait}\033[0m{' ' * (W - len(wait) - 3)}\033[1;36m\u2551\033[0m\n"
            out += f"\033[1;36m\u2551\033[0m{' ' * W}\033[1;36m\u2551\033[0m\n"
        else:
            m = self._msg
            for label, attr, unit, fmt, scale, color in FIELDS:
                val = getattr(m, attr) * scale
                val_str = fmt.format(val)
                graph = sparkline(self._history[attr], color=color)
                # Visible chars: 2 + 15 + 10 + 1 + 3 + 1 + GRAPH_CHARS = 52
                vis = 2 + 15 + 10 + 1 + 3 + 1 + GRAPH_CHARS
                pad = W - vis
                out += (
                    f"\033[1;36m\u2551\033[0m"
                    f"  {label:<15s}{val_str} {unit:<3s} {graph}"
                    f"{' ' * pad}\033[1;36m\u2551\033[0m\n"
                )

            out += f"\033[1;36m\u2560{bar}\u2563\033[0m\n"
            status = f"  Rate: {rate:5.1f} Hz   Msgs: {self._count}"
            out += f"\033[1;36m\u2551\033[0m\033[90m{status:<{W}s}\033[0m\033[1;36m\u2551\033[0m\n"

            if self._last_time and (time.monotonic() - self._last_time) > 2.0:
                stale = "  [STALE]"
                out += f"\033[1;36m\u2551\033[0m  \033[31m{stale}\033[0m{' ' * (W - len(stale) - 2)}\033[1;36m\u2551\033[0m\n"

        out += f"\033[1;36m\u255a{bar}\u255d\033[0m\n"
        out += "\033[90m  Ctrl+C to exit\033[0m\033[K\n"

        sys.stdout.write(out)
        sys.stdout.flush()

    def destroy_node(self):
        sys.stdout.write("\033[?25h\n")
        sys.stdout.flush()
        super().destroy_node()


def main():
    rclpy.init()
    node = PowerDashboard()

    def shutdown(*_):
        node.destroy_node()
        rclpy.try_shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
