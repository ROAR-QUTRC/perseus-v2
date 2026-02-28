#!/usr/bin/python3
"""
Pico USB Monitor TUI — live dashboard for the Space Resources End Effector.

Connects over USB serial, sends the 'stream' command, and renders a curses
dashboard from the JSON status lines the firmware emits at 5 Hz.

Requirements: pyserial  (pip install pyserial)
"""

import argparse
import curses
import json
import sys
import threading
import time

import serial

# Command type names matching protocol.hpp
CMD_NAMES = {
    0x01: "SET_SERVO",
    0x02: "SET_HEATER",
    0x03: "SET_ALL",
    0x04: "GET_STATUS",
    0xFE: "STOP_ALL",
    0xFF: "HEARTBEAT",
}


def parse_args():
    p = argparse.ArgumentParser(description="Pico USB Monitor TUI")
    p.add_argument("--port", default="/dev/ttyACM0", help="Serial port (default: /dev/ttyACM0)")
    p.add_argument("--baud", type=int, default=115200, help="Baud rate (default: 115200)")
    return p.parse_args()


class SerialReader:
    """Background thread that reads JSON lines from the Pico and updates shared state."""

    def __init__(self, port: str, baud: int):
        self.port = port
        self.baud = baud
        self.lock = threading.Lock()
        self.data: dict = {}
        self.connected = False
        self.error = ""
        self.hz = 0.0
        self._ser: serial.Serial | None = None
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._timestamps: list[float] = []

    def start(self):
        self._thread.start()

    def stop(self):
        self._stop.set()
        if self._ser and self._ser.is_open:
            try:
                self._ser.write(b"stop\n")
                self._ser.flush()
            except Exception:
                pass
            try:
                self._ser.close()
            except Exception:
                pass

    def snapshot(self) -> tuple[dict, bool, str, float]:
        with self.lock:
            return dict(self.data), self.connected, self.error, self.hz

    def _run(self):
        while not self._stop.is_set():
            try:
                self._ser = serial.Serial(self.port, self.baud, timeout=1)
                with self.lock:
                    self.connected = True
                    self.error = ""
                # Send stream command
                time.sleep(0.2)
                self._ser.write(b"stream\n")
                self._ser.flush()

                while not self._stop.is_set():
                    line = self._ser.readline()
                    if not line:
                        continue
                    line = line.decode("utf-8", errors="replace").strip()
                    if not line.startswith("{"):
                        continue
                    try:
                        obj = json.loads(line)
                    except json.JSONDecodeError:
                        continue

                    now = time.monotonic()
                    with self.lock:
                        self.data = obj
                        self._timestamps.append(now)
                        # Keep last 2 seconds of timestamps for Hz calc
                        cutoff = now - 2.0
                        self._timestamps = [t for t in self._timestamps if t > cutoff]
                        if len(self._timestamps) >= 2:
                            span = self._timestamps[-1] - self._timestamps[0]
                            self.hz = (len(self._timestamps) - 1) / span if span > 0 else 0
                        else:
                            self.hz = 0.0

            except serial.SerialException as e:
                with self.lock:
                    self.connected = False
                    self.error = str(e)
                    self.data = {}
                    self.hz = 0.0
                time.sleep(1.0)
            except Exception as e:
                with self.lock:
                    self.connected = False
                    self.error = str(e)
                time.sleep(1.0)


def bar(value: float, max_val: float, width: int) -> str:
    """Render a simple bar graph."""
    if max_val <= 0:
        ratio = 0.0
    else:
        ratio = max(0.0, min(1.0, abs(value) / max_val))
    filled = int(ratio * width)
    return "\u2588" * filled + "\u2591" * (width - filled)


def format_uptime(ms: int) -> str:
    secs = ms // 1000
    h = secs // 3600
    m = (secs % 3600) // 60
    s = secs % 60
    return f"{h:02d}:{m:02d}:{s:02d}"


def rssi_bar(rssi: int) -> str:
    """RSSI bar: -120 dBm = empty, -30 dBm = full."""
    clamped = max(-120, min(-30, rssi))
    ratio = (clamped + 120) / 90.0
    width = 6
    filled = int(ratio * width)
    return "\u2588" * filled + "\u2591" * (width - filled)


def draw(stdscr, reader: SerialReader):
    curses.curs_set(0)
    stdscr.nodelay(True)

    # Set up colors
    curses.start_color()
    curses.use_default_colors()
    curses.init_pair(1, curses.COLOR_GREEN, -1)   # OK / connected
    curses.init_pair(2, curses.COLOR_RED, -1)      # fault / disconnected
    curses.init_pair(3, curses.COLOR_YELLOW, -1)   # warning
    curses.init_pair(4, curses.COLOR_CYAN, -1)     # labels

    GREEN = curses.color_pair(1)
    RED = curses.color_pair(2)
    YELLOW = curses.color_pair(3)
    CYAN = curses.color_pair(4)
    BOLD = curses.A_BOLD

    BOX_W = 52

    while True:
        ch = stdscr.getch()
        if ch in (ord("q"), ord("Q")):
            break

        data, connected, error, hz = reader.snapshot()
        stdscr.erase()
        height, width = stdscr.getmaxyx()

        if width < BOX_W + 2 or height < 22:
            stdscr.addstr(0, 0, "Terminal too small (need 54x22)")
            stdscr.refresh()
            time.sleep(0.2)
            continue

        row = 0

        def hline(r, label):
            nonlocal row
            try:
                stdscr.addstr(r, 0, f"\u250c\u2500 {label} ", CYAN | BOLD)
                remaining = BOX_W - 4 - len(label)
                stdscr.addstr("\u2500" * remaining + "\u2510")
            except curses.error:
                pass
            return r + 1

        def bline(r):
            try:
                stdscr.addstr(r, 0, "\u2514" + "\u2500" * (BOX_W - 2) + "\u2518")
            except curses.error:
                pass
            return r + 1

        def mline(r, text, attr=0):
            try:
                line = f"\u2502 {text}"
                line = line.ljust(BOX_W - 1) + "\u2502"
                stdscr.addstr(r, 0, line, attr)
            except curses.error:
                pass
            return r + 1

        if not connected:
            row = hline(row, "Connection")
            if error:
                row = mline(row, f"ERROR: {error[:44]}", RED)
            else:
                row = mline(row, "Connecting...", YELLOW)
            row = bline(row)
            stdscr.addstr(row + 1, 2, f"USB: {reader.port}", CYAN)
            stdscr.refresh()
            time.sleep(0.2)
            continue

        if not data:
            row = hline(row, "Connection")
            row = mline(row, "Waiting for data...", YELLOW)
            row = bline(row)
            stdscr.refresh()
            time.sleep(0.2)
            continue

        # --- Connection box ---
        row = hline(row, "Connection")

        radio = data.get("radio", False)
        rssi = data.get("rssi", 0)
        uptime = data.get("up", 0)
        cmd_age = data.get("cmd_age", 0xFFFFFFFF)

        radio_str = "CONNECTED" if radio else "DISCONNECTED"
        radio_color = GREEN if radio else RED
        try:
            stdscr.addstr(row, 0, "\u2502 ")
            stdscr.addstr("Radio Link:  ", CYAN)
            stdscr.addstr(f"{radio_str:14s}", radio_color | BOLD)
            if radio:
                stdscr.addstr("RSSI: ", CYAN)
                stdscr.addstr(f"{rssi:4d} dBm  ", 0)
                stdscr.addstr(rssi_bar(rssi), GREEN if rssi > -70 else YELLOW if rssi > -90 else RED)
            stdscr.addstr(row, BOX_W - 1, "\u2502")
        except curses.error:
            pass
        row += 1

        age_str = f"{cmd_age} ms" if cmd_age != 0xFFFFFFFF else "never"
        row = mline(row, f"Uptime:      {format_uptime(uptime)}     Cmd Age: {age_str}")
        row = bline(row)

        # --- Telemetry box ---
        row = hline(row, "Telemetry")

        servo = data.get("servo", 0)
        servo_f = servo / 1000.0
        heater = data.get("heater", 0)
        heater_pct = heater * 100 / 255
        current_ma = data.get("ma", 0)
        current_a = current_ma / 1000.0
        bar_w = 26

        row = mline(row, f"Servo Speed: {servo_f:+6.3f}  {bar(abs(servo_f), 1.0, bar_w)}")
        row = mline(row, f"Heater Duty:   {heater_pct:4.0f}%  {bar(heater_pct, 100, bar_w)}")
        row = mline(row, f"Current:     {current_a:5.2f} A {bar(current_a, 5.0, bar_w)}")
        row = bline(row)

        # --- Faults box ---
        row = hline(row, "Faults")

        wd = data.get("wd", False)
        servo_ok = data.get("servo_ok", True)
        overcurrent = current_ma > 10000

        def fault_line(label, ok):
            status = "OK" if ok else "FAULT"
            color = GREEN if ok else RED | BOLD
            try:
                stdscr.addstr(row, 0, "\u2502 ")
                stdscr.addstr(f"{label:20s}", CYAN)
                stdscr.addstr(f"{status}", color)
                stdscr.addstr(row, BOX_W - 1, "\u2502")
            except curses.error:
                pass

        fault_line("Overcurrent:", not overcurrent)
        row += 1
        fault_line("Comm Timeout:", not wd)
        row += 1
        fault_line("Servo Driver:", servo_ok)
        row += 1
        row = bline(row)

        # --- Radio Traffic box ---
        row = hline(row, "Radio Traffic")

        rx = data.get("rx", 0)
        tx = data.get("tx", 0)
        bad = data.get("bad", 0)
        last_cmd = data.get("last_cmd", 0)
        cmd_name = CMD_NAMES.get(last_cmd, f"0x{last_cmd:02X}")

        row = mline(row, f"Commands RX:  {rx:<10d}  Last: {cmd_name}")
        row = mline(row, f"Telemetry TX: {tx:<10d}  Invalid RX: {bad}")
        row = bline(row)

        # --- Footer ---
        try:
            stdscr.addstr(row, 2, f"USB: {reader.port}", CYAN)
            stdscr.addstr(f"  Stream: {hz:.1f} Hz")
            row += 1
            stdscr.addstr(row, 2, "[q] Quit", BOLD)
        except curses.error:
            pass

        stdscr.refresh()
        time.sleep(0.2)


def main():
    args = parse_args()
    reader = SerialReader(args.port, args.baud)
    reader.start()
    try:
        curses.wrapper(lambda stdscr: draw(stdscr, reader))
    except KeyboardInterrupt:
        pass
    finally:
        reader.stop()


if __name__ == "__main__":
    main()
