"""Curses-based setup TUI for configuring mapping autotune before launch.

Provides an interactive screen to configure session name, max runs, settling
time, maneuver pattern, speeds, and phase enable/disable before the autotune
begins driving the robot.
"""

import curses
import time

from mapping_autotune.param_manager import ParamManager


# Maneuver pattern definitions: (key, display_name, description)
MANEUVER_PATTERNS = [
    ("straight", "Straight", "fwd/back 1m"),
    ("small_turn", "Small turn", "fwd + 45\u00b0 turn"),
    ("big_turn", "Big turn", "fwd + 90\u00b0 turn"),
    ("box_return", "Box return", "fwd, 180\u00b0, back"),
    ("corridor", "Corridor", "2m x 1m rectangle"),
]

# Phase definitions for display: (phase_num, display_name)
PHASE_DISPLAY = [
    (1, "IMU integration"),
    (2, "SLAM rotation"),
    (3, "Scan matching"),
    (4, "Timing"),
    (5, "Speed limits"),
    (6, "EKF noise"),
]


class SetupTUI:
    """Interactive terminal UI for configuring mapping autotune sessions."""

    def __init__(self, defaults=None):
        """Initialize with optional default values.

        Args:
            defaults: Optional dict with initial values for any config field.
        """
        if defaults is None:
            defaults = {}

        self._session_name = defaults.get(
            "session_name", time.strftime("autotune_%Y%m%d_%H%M%S")
        )
        self._max_runs = defaults.get("max_runs", 10)
        self._settling_time = defaults.get("settling_time", 3.0)
        self._maneuver_pattern_idx = 0
        self._linear_speed = defaults.get("linear_speed", 0.2)
        self._rotation_speed = defaults.get("rotation_speed", 0.5)

        # Set initial maneuver pattern from defaults
        pattern = defaults.get("maneuver_pattern", "straight")
        for i, (key, _, _) in enumerate(MANEUVER_PATTERNS):
            if key == pattern:
                self._maneuver_pattern_idx = i
                break

        # Phase enable flags (all enabled by default)
        default_enabled = defaults.get("enabled_phases", [1, 2, 3, 4, 5, 6])
        self._phase_enabled = {}
        for phase_num, _ in PHASE_DISPLAY:
            self._phase_enabled[phase_num] = phase_num in default_enabled

        # Navigation state
        self._fields = self._build_field_list()
        self._cursor = 0
        self._editing_name = False

    def _build_field_list(self):
        """Build the ordered list of navigable fields.

        Returns:
            List of (field_type, identifier) tuples.
        """
        fields = [
            ("session_name", None),
            ("max_runs", None),
            ("settling_time", None),
        ]
        for i in range(len(MANEUVER_PATTERNS)):
            fields.append(("maneuver", i))
        fields.append(("linear_speed", None))
        fields.append(("rotation_speed", None))
        for phase_num, _ in PHASE_DISPLAY:
            fields.append(("phase", phase_num))
        fields.append(("confirm", None))
        return fields

    def run(self, stdscr=None):
        """Launch the TUI and return the config dict or None.

        Args:
            stdscr: Optional curses window (for testing). If None, uses
                     curses.wrapper() to create one.

        Returns:
            Config dict if user confirms, None if user quits.
        """
        if stdscr is not None:
            return self._main_loop(stdscr)
        return curses.wrapper(self._main_loop)

    def _main_loop(self, stdscr):
        """Set up curses environment and run the input/draw loop."""
        curses.start_color()
        curses.use_default_colors()
        curses.init_pair(1, curses.COLOR_GREEN, curses.COLOR_BLACK)
        curses.init_pair(2, curses.COLOR_RED, curses.COLOR_BLACK)
        curses.init_pair(3, curses.COLOR_YELLOW, curses.COLOR_BLACK)
        curses.init_pair(4, curses.COLOR_CYAN, curses.COLOR_BLACK)
        curses.init_pair(5, curses.COLOR_BLACK, curses.COLOR_WHITE)

        stdscr.keypad(True)
        curses.curs_set(0)
        stdscr.nodelay(False)

        while True:
            stdscr.erase()
            self._draw(stdscr)
            stdscr.refresh()

            key = stdscr.getch()
            result = self._handle_input(key, stdscr)
            if result == "confirm":
                return self._build_config()
            elif result == "quit":
                return None

    # ── Drawing ────────────────────────────────────────────────────────

    def _draw(self, stdscr):
        """Draw the full setup screen."""
        max_y, max_x = stdscr.getmaxyx()
        row = 0

        # Title bar
        title = " MAPPING AUTOTUNE SETUP "
        bar_width = min(max_x - 1, 50)
        padding = max(0, (bar_width - len(title)) // 2)
        title_line = "\u2500" * padding + title + "\u2500" * padding
        self._addstr_safe(
            stdscr, row, 1, title_line, curses.color_pair(4) | curses.A_BOLD
        )
        row += 2

        field_idx = 0

        # ── Session settings ──────────────────────────────────────────
        self._addstr_safe(
            stdscr,
            row,
            2,
            "\u2500\u2500 Session Settings \u2500" * 2,
            curses.color_pair(4),
        )
        row += 1

        # Session name
        row = self._draw_field(
            stdscr, row, field_idx, "Session name:", self._session_name
        )
        field_idx += 1

        # Max runs
        row = self._draw_field(
            stdscr, row, field_idx, "Max runs:", str(self._max_runs), adjustable=True
        )
        field_idx += 1

        # Settling time
        row = self._draw_field(
            stdscr,
            row,
            field_idx,
            "Settling time:",
            f"{self._settling_time:.1f}s",
            adjustable=True,
        )
        field_idx += 1

        row += 1

        # ── Maneuver pattern ──────────────────────────────────────────
        self._addstr_safe(
            stdscr,
            row,
            2,
            "\u2500\u2500 Maneuver Pattern \u2500" * 2,
            curses.color_pair(4),
        )
        row += 1

        for i, (key, name, desc) in enumerate(MANEUVER_PATTERNS):
            if row >= max_y - 8:
                break
            is_selected = i == self._maneuver_pattern_idx
            is_cursor = self._cursor == field_idx

            marker = "(\u2022)" if is_selected else "( )"
            prefix = "> " if is_cursor else "  "
            line = f"{prefix}{marker} {name} ({desc})"

            attr = curses.color_pair(5) if is_cursor else 0
            if is_selected and not is_cursor:
                attr = curses.color_pair(1)
            self._addstr_safe(stdscr, row, 2, line, attr)
            row += 1
            field_idx += 1

        row += 1

        # ── Speeds ────────────────────────────────────────────────────
        self._addstr_safe(
            stdscr,
            row,
            2,
            "\u2500\u2500 Speeds \u2500" * 4,
            curses.color_pair(4),
        )
        row += 1

        row = self._draw_field(
            stdscr,
            row,
            field_idx,
            "Linear speed:",
            f"{self._linear_speed:.2f} m/s",
            adjustable=True,
        )
        field_idx += 1

        row = self._draw_field(
            stdscr,
            row,
            field_idx,
            "Rotation speed:",
            f"{self._rotation_speed:.1f} rad/s",
            adjustable=True,
        )
        field_idx += 1

        row += 1

        # ── Phases ────────────────────────────────────────────────────
        self._addstr_safe(
            stdscr,
            row,
            2,
            "\u2500\u2500 Phases \u2500" * 4,
            curses.color_pair(4),
        )
        row += 1

        # Compute allocation for display
        allocation = self._compute_allocation()

        for phase_num, phase_name in PHASE_DISPLAY:
            if row >= max_y - 5:
                break
            is_cursor = self._cursor == field_idx
            enabled = self._phase_enabled[phase_num]

            check = "[x]" if enabled else "[ ]"
            prefix = "> " if is_cursor else "  "

            runs = allocation.get(phase_num, 0)
            if enabled and runs > 0:
                run_info = f"({runs} runs)"
            elif enabled:
                run_info = "(0 runs)"
            else:
                run_info = " \u2500\u2500"

            line = f"{prefix}{check} {phase_num}. {phase_name:<18} {run_info}"

            attr = curses.color_pair(5) if is_cursor else 0
            if enabled and not is_cursor:
                attr = curses.color_pair(1)
            self._addstr_safe(stdscr, row, 2, line, attr)
            row += 1
            field_idx += 1

        row += 1

        # ── Total summary ─────────────────────────────────────────────
        total_runs = sum(
            allocation.get(p, 0)
            for p in self._phase_enabled
            if self._phase_enabled.get(p)
        )
        self._addstr_safe(
            stdscr,
            row,
            2,
            f"Total: {total_runs} runs",
            curses.color_pair(3) | curses.A_BOLD,
        )
        row += 2

        # ── Confirm button ────────────────────────────────────────────
        is_cursor = self._cursor == field_idx
        confirm_text = "[ Confirm & Start ]"
        attr = (
            curses.color_pair(5) | curses.A_BOLD if is_cursor else curses.color_pair(1)
        )
        prefix = "> " if is_cursor else "  "
        self._addstr_safe(
            stdscr, min(row, max_y - 4), 2, f"{prefix}{confirm_text}", attr
        )
        row += 2

        # ── Key hints ─────────────────────────────────────────────────
        hints = "[Enter] Confirm  [q] Quit  [\u2191\u2193] Navigate  [Space] Toggle  [\u2190\u2192] Adjust"
        self._addstr_safe(stdscr, min(row, max_y - 1), 1, hints, curses.color_pair(3))

    def _draw_field(self, stdscr, row, field_idx, label, value, adjustable=False):
        """Draw a labeled field with cursor highlight.

        Returns:
            The next row number.
        """
        is_cursor = self._cursor == field_idx
        prefix = "> " if is_cursor else "  "

        arrows = " \u25c0\u25b6" if adjustable and is_cursor else ""
        line = f"{prefix}{label:<18} {value}{arrows}"

        attr = curses.color_pair(5) if is_cursor else 0
        self._addstr_safe(stdscr, row, 2, line, attr)
        return row + 1

    # ── Input handling ─────────────────────────────────────────────────

    def _handle_input(self, key, stdscr):
        """Process a keypress.

        Returns:
            'confirm' to start, 'quit' to exit, or None to continue.
        """
        if self._editing_name:
            return self._handle_name_edit(key, stdscr)

        if key == ord("q"):
            return "quit"

        if key == curses.KEY_UP:
            if self._cursor > 0:
                self._cursor -= 1
        elif key == curses.KEY_DOWN:
            if self._cursor < len(self._fields) - 1:
                self._cursor += 1
        elif key in (curses.KEY_ENTER, 10, 13):
            return self._handle_enter(stdscr)
        elif key == ord(" "):
            self._handle_space()
        elif key == curses.KEY_LEFT:
            self._handle_adjust(-1)
        elif key == curses.KEY_RIGHT:
            self._handle_adjust(1)

        return None

    def _handle_enter(self, stdscr):
        """Handle Enter key on the current field."""
        field_type, field_id = self._fields[self._cursor]

        if field_type == "session_name":
            self._edit_session_name(stdscr)
            return None
        elif field_type == "confirm":
            return "confirm"
        elif field_type == "maneuver":
            self._maneuver_pattern_idx = field_id
            return None
        elif field_type == "phase":
            self._phase_enabled[field_id] = not self._phase_enabled[field_id]
            return None

        return None

    def _handle_space(self):
        """Handle Space key (toggle) on the current field."""
        field_type, field_id = self._fields[self._cursor]

        if field_type == "maneuver":
            self._maneuver_pattern_idx = field_id
        elif field_type == "phase":
            self._phase_enabled[field_id] = not self._phase_enabled[field_id]

    def _handle_adjust(self, direction):
        """Handle left/right arrow key to adjust numeric values.

        Args:
            direction: -1 for left (decrease), +1 for right (increase).
        """
        field_type, _ = self._fields[self._cursor]

        if field_type == "max_runs":
            self._max_runs = max(1, min(100, self._max_runs + direction))
        elif field_type == "settling_time":
            self._settling_time = max(
                0.5, min(30.0, self._settling_time + direction * 0.5)
            )
            self._settling_time = round(self._settling_time * 2) / 2  # snap to 0.5
        elif field_type == "linear_speed":
            self._linear_speed = max(
                0.05, min(0.5, self._linear_speed + direction * 0.05)
            )
            self._linear_speed = round(self._linear_speed * 20) / 20  # snap to 0.05
        elif field_type == "rotation_speed":
            self._rotation_speed = max(
                0.1, min(1.5, self._rotation_speed + direction * 0.1)
            )
            self._rotation_speed = round(self._rotation_speed * 10) / 10  # snap to 0.1

    # ── Session name editing ───────────────────────────────────────────

    def _edit_session_name(self, stdscr):
        """Open inline text editing for the session name."""
        max_y, max_x = stdscr.getmaxyx()
        input_row = max_y - 2
        prompt = "Session name: "

        curses.curs_set(1)
        stdscr.move(input_row, 0)
        stdscr.clrtoeol()
        self._addstr_safe(stdscr, input_row, 1, prompt, curses.A_BOLD)

        text = list(self._session_name)
        cursor_pos = len(prompt) + 1 + len(text)

        # Show current text
        self._addstr_safe(stdscr, input_row, len(prompt) + 1, self._session_name)
        stdscr.refresh()

        while True:
            ch = stdscr.getch()
            if ch in (curses.KEY_ENTER, 10, 13):
                self._session_name = "".join(text)
                break
            elif ch == 27:  # Escape — cancel
                break
            elif ch in (curses.KEY_BACKSPACE, 127, 8):
                if text:
                    text.pop()
                    cursor_pos -= 1
                    stdscr.move(input_row, cursor_pos)
                    stdscr.delch()
            elif 32 <= ch < 127:
                if cursor_pos < max_x - 2:
                    text.append(chr(ch))
                    self._addstr_safe(stdscr, input_row, cursor_pos, chr(ch))
                    cursor_pos += 1
            stdscr.refresh()

        curses.curs_set(0)

    def _handle_name_edit(self, key, stdscr):
        """Handle keystrokes during name editing mode (unused — inline editing)."""
        return None

    # ── Configuration building ─────────────────────────────────────────

    def _compute_allocation(self):
        """Compute the run allocation for currently enabled phases."""
        enabled = [p for p, en in self._phase_enabled.items() if en]
        if not enabled:
            return {p: 0 for p, _ in PHASE_DISPLAY}
        return ParamManager.allocate_runs(self._max_runs, enabled_phases=enabled)

    def _build_config(self):
        """Build the config dict from current TUI state.

        Returns:
            Configuration dict.
        """
        pattern_key, _, _ = MANEUVER_PATTERNS[self._maneuver_pattern_idx]
        enabled = [p for p, en in self._phase_enabled.items() if en]

        return {
            "session_name": self._session_name,
            "max_runs": self._max_runs,
            "settling_time": self._settling_time,
            "maneuver_pattern": pattern_key,
            "linear_speed": self._linear_speed,
            "rotation_speed": self._rotation_speed,
            "enabled_phases": sorted(enabled),
        }

    # ── Safe curses output ─────────────────────────────────────────────

    @staticmethod
    def _addstr_safe(stdscr, row, col, text, attr=0):
        """Write text to stdscr, silently truncating if it goes out of bounds."""
        max_y, max_x = stdscr.getmaxyx()
        if row < 0 or row >= max_y or col >= max_x:
            return
        available = max_x - col - 1
        if available <= 0:
            return
        truncated = text[:available]
        try:
            stdscr.addstr(row, col, truncated, attr)
        except curses.error:
            pass


def main():
    """Standalone entry point for the setup TUI."""
    tui = SetupTUI()
    config = tui.run()
    if config is not None:
        import json

        print("Configuration:")
        print(json.dumps(config, indent=2))
    else:
        print("Setup cancelled.")
