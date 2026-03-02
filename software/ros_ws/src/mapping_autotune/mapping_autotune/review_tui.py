"""Curses-based TUI for reviewing mapping autotune results.

Provides interactive session/run browsing, rating, comparison, and YAML
export backed by the autotune SQLite database.
"""

import curses
import json
import os

import rclpy
import yaml

from mapping_autotune.db_manager import DbManager


# ASCII brightness scale from darkest to lightest
_ASCII_CHARS = " .:-=+*#%@"


class ReviewTui:
    """Interactive terminal UI for reviewing mapping autotune sessions."""

    def __init__(self, db_path):
        self._db = DbManager(db_path)
        self._sessions = self._db.get_all_sessions()
        self._runs = []
        self._detailed_runs = {}

        self._selected_session_idx = 0
        self._selected_run_idx = 0
        self._mode = "sessions"

        # For side-by-side comparison
        self._compare_run_id = None

        # Preload runs for each session
        self._session_runs = {}
        for session in self._sessions:
            sid = session["id"]
            runs = self._db.get_runs(sid)
            self._session_runs[sid] = runs

    # ------------------------------------------------------------------
    # Public entry point
    # ------------------------------------------------------------------

    def run(self):
        """Launch the curses TUI."""
        curses.wrapper(self._main_loop)

    # ------------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------------

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

            if self._mode == "sessions":
                self._draw_sessions(stdscr)
            elif self._mode == "runs":
                self._draw_runs(stdscr)
            elif self._mode == "detail":
                self._draw_detail(stdscr)
            elif self._mode == "compare":
                self._draw_compare(stdscr)

            stdscr.refresh()
            key = stdscr.getch()

            if not self._handle_input(key, stdscr):
                break

    # ------------------------------------------------------------------
    # Input handling
    # ------------------------------------------------------------------

    def _handle_input(self, key, stdscr):
        """Dispatch key events based on the current mode.

        Returns False when the TUI should exit.
        """
        if key == ord("q"):
            return False

        if self._mode == "sessions":
            return self._handle_sessions_input(key, stdscr)
        elif self._mode == "runs":
            return self._handle_runs_input(key, stdscr)
        elif self._mode == "detail":
            return self._handle_detail_input(key, stdscr)
        elif self._mode == "compare":
            return self._handle_compare_input(key)

        return True

    def _handle_sessions_input(self, key, stdscr):
        if key == curses.KEY_UP and self._selected_session_idx > 0:
            self._selected_session_idx -= 1
        elif (
            key == curses.KEY_DOWN
            and self._selected_session_idx < len(self._sessions) - 1
        ):
            self._selected_session_idx += 1
        elif key in (curses.KEY_ENTER, 10, 13):
            if self._sessions:
                self._enter_runs_mode()
        elif key == ord("r"):
            if self._sessions:
                self._export_report(stdscr)
        return True

    def _handle_runs_input(self, key, stdscr):
        if key == curses.KEY_UP and self._selected_run_idx > 0:
            self._selected_run_idx -= 1
        elif key == curses.KEY_DOWN and self._selected_run_idx < len(self._runs) - 1:
            self._selected_run_idx += 1
        elif key in (curses.KEY_ENTER, 10, 13):
            if self._runs:
                self._mode = "detail"
        elif key in (ord("1"), ord("2"), ord("3"), ord("4"), ord("5")):
            if self._runs:
                self._rate_run(key - ord("0"))
        elif key == ord("n"):
            if self._runs:
                self._add_notes(stdscr)
        elif key == ord("c"):
            if self._runs:
                self._handle_compare_mark()
        elif key == ord("e"):
            self._export_best_yaml(stdscr)
        elif key == 27:  # Escape
            self._mode = "sessions"
            self._compare_run_id = None
        return True

    def _handle_detail_input(self, key, stdscr):
        if key in (ord("1"), ord("2"), ord("3"), ord("4"), ord("5")):
            if self._runs:
                self._rate_run(key - ord("0"))
        elif key == ord("n"):
            if self._runs:
                self._add_notes(stdscr)
        elif key == 27:  # Escape
            self._mode = "runs"
        return True

    def _handle_compare_input(self, key):
        if key == 27:  # Escape
            self._mode = "runs"
            self._compare_run_id = None
        return True

    # ------------------------------------------------------------------
    # Drawing: sessions
    # ------------------------------------------------------------------

    def _draw_sessions(self, stdscr):
        max_y, max_x = stdscr.getmaxyx()
        row = 0

        self._addstr_safe(
            stdscr,
            row,
            0,
            "=== MAPPING AUTOTUNE REVIEW ===",
            curses.color_pair(4) | curses.A_BOLD,
        )
        row += 1

        self._addstr_safe(stdscr, row, 0, "Sessions:", curses.A_BOLD)
        row += 1

        header = f"  {'#':>3} | {'Name':<20} | {'Date':<19} | {'Runs':>4} | {'Best Score':>10} | {'Status':<10}"
        self._addstr_safe(stdscr, row, 0, header, curses.color_pair(4))
        row += 1

        sep = "  " + "-" * min(max_x - 4, 90)
        self._addstr_safe(stdscr, row, 0, sep)
        row += 1

        for idx, session in enumerate(self._sessions):
            if row >= max_y - 2:
                break

            sid = session["id"]
            name = session.get("name", "")[:20]
            date = (session.get("created_at") or "")[:19]
            runs = self._session_runs.get(sid, [])
            num_runs = len(runs)
            status = session.get("status", "")

            # Find best score for this session
            best_run = self._db.get_best_run(sid)
            best_score = (
                f"{best_run['composite_score']:.2f}"
                if best_run and best_run.get("composite_score") is not None
                else "  --"
            )

            prefix = "> " if idx == self._selected_session_idx else "  "
            line = f"{prefix}{idx + 1:>3} | {name:<20} | {date:<19} | {num_runs:>4} | {best_score:>10} | {status:<10}"

            attr = curses.color_pair(5) if idx == self._selected_session_idx else 0
            self._addstr_safe(stdscr, row, 0, line, attr)
            row += 1

        row += 1
        self._addstr_safe(
            stdscr,
            min(row, max_y - 1),
            0,
            "Keys: Enter=view runs, r=export report, q=quit",
            curses.color_pair(3),
        )

    # ------------------------------------------------------------------
    # Drawing: runs
    # ------------------------------------------------------------------

    def _draw_runs(self, stdscr):
        max_y, max_x = stdscr.getmaxyx()
        row = 0

        if not self._sessions:
            self._addstr_safe(stdscr, row, 0, "No sessions found.")
            return

        session = self._sessions[self._selected_session_idx]
        num_runs = len(self._runs)

        title = f"=== Session: {session.get('name', '')} ({num_runs} runs) ==="
        self._addstr_safe(stdscr, row, 0, title, curses.color_pair(4) | curses.A_BOLD)
        row += 1

        header = (
            f"  {'#':>3} | {'Composite':>9} | {'Wall':>5} | {'Thick':>5} | "
            f"{'Ghost':>5} | {'Symm':>5} | {'Free':>5} | {'Dens':>5} | "
            f"{'Rating':>6} | Params Changed"
        )
        self._addstr_safe(stdscr, row, 0, header, curses.color_pair(4))
        row += 1

        sep = "  " + "-" * min(max_x - 4, 110)
        self._addstr_safe(stdscr, row, 0, sep)
        row += 1

        # Parse baseline params for diffing
        base_slam, base_ekf = self._get_baseline_params(session)

        for idx, run in enumerate(self._runs):
            if row >= max_y - 2:
                break

            run_id = run["id"]
            detail = self._get_detailed_run(run_id)

            composite = detail.get("composite_score")
            wall = detail.get("wall_straightness")
            thick = detail.get("wall_thickness")
            ghost = detail.get("ghost_wall_score")
            symm = detail.get("symmetry_score")
            free = detail.get("free_space_consistency")
            dens = detail.get("occupied_density_score")
            rating = detail.get("rating")

            rating_str = f"{rating}/5" if rating is not None else " --"

            # Determine changed params
            changed_str = self._get_changed_params_str(detail, base_slam, base_ekf)

            prefix = "> " if idx == self._selected_run_idx else "  "

            # Build the line piece by piece with coloring
            line_prefix = f"{prefix}{run.get('run_number', idx + 1):>3} | "
            self._addstr_safe(
                stdscr,
                row,
                0,
                line_prefix,
                curses.color_pair(5) if idx == self._selected_run_idx else 0,
            )

            col = len(line_prefix)
            for val in [composite, wall, thick, ghost, symm, free, dens]:
                val_str = f"{val:>9.2f}" if val is not None else "     --  "
                if len(val_str) < 9:
                    val_str = val_str.rjust(9)
                # Trim to consistent width for non-composite columns
                if val == composite:
                    display = val_str
                else:
                    display = f"{val:>5.2f}" if val is not None else "  -- "

                color = self._score_color(val)
                if val == composite:
                    self._addstr_safe(stdscr, row, col, f"{display} | ", color)
                    col += len(f"{display} | ")
                else:
                    self._addstr_safe(stdscr, row, col, f"{display} | ", color)
                    col += len(f"{display} | ")

            self._addstr_safe(
                stdscr,
                row,
                col,
                f"{rating_str:>6} | {changed_str}",
                curses.color_pair(5) if idx == self._selected_run_idx else 0,
            )

            row += 1

        row += 1
        keys = "Keys: Enter=detail, 1-5=rate, n=notes, c=compare, e=export YAML, Esc=back, q=quit"
        self._addstr_safe(stdscr, min(row, max_y - 1), 0, keys, curses.color_pair(3))

    # ------------------------------------------------------------------
    # Drawing: run detail
    # ------------------------------------------------------------------

    def _draw_detail(self, stdscr):
        max_y, max_x = stdscr.getmaxyx()
        row = 0

        if not self._runs:
            self._addstr_safe(stdscr, row, 0, "No runs available.")
            return

        run = self._runs[self._selected_run_idx]
        run_id = run["id"]
        detail = self._get_detailed_run(run_id)
        run_num = run.get("run_number", self._selected_run_idx + 1)

        composite = detail.get("composite_score")
        rating = detail.get("rating")
        status = detail.get("status", "unknown")
        rating_str = f"{rating}/5" if rating is not None else "--"
        composite_str = f"{composite:.2f}" if composite is not None else "--"

        title = f"=== Run #{run_num} Detail ==="
        self._addstr_safe(stdscr, row, 0, title, curses.color_pair(4) | curses.A_BOLD)
        row += 1

        status_line = (
            f"Status: {status} | Composite: {composite_str} | Rating: {rating_str}"
        )
        self._addstr_safe(stdscr, row, 0, status_line)
        row += 2

        # Parameter diff from baseline
        session = self._sessions[self._selected_session_idx]
        base_slam, base_ekf = self._get_baseline_params(session)
        run_slam = self._parse_json_safe(detail.get("slam_params", "{}"))
        run_ekf = self._parse_json_safe(detail.get("ekf_params", "{}"))

        self._addstr_safe(
            stdscr, row, 0, "Parameters (diff from baseline):", curses.A_BOLD
        )
        row += 1

        diff_found = False
        for key, value in run_slam.items():
            if row >= max_y - 10:
                break
            baseline_val = base_slam.get(key)
            if baseline_val != value:
                line = f"  {key}: {value} (was {baseline_val})"
                self._addstr_safe(stdscr, row, 0, line, curses.color_pair(3))
                row += 1
                diff_found = True

        for key, value in run_ekf.items():
            if row >= max_y - 10:
                break
            baseline_val = base_ekf.get(key)
            if baseline_val != value:
                line = f"  {key}: {value} (was {baseline_val})"
                self._addstr_safe(stdscr, row, 0, line, curses.color_pair(3))
                row += 1
                diff_found = True

        if not diff_found:
            self._addstr_safe(stdscr, row, 0, "  (baseline -- no changes)")
            row += 1

        row += 1

        # Metrics with bar charts
        self._addstr_safe(stdscr, row, 0, "Metrics:", curses.A_BOLD)
        row += 1

        metrics = [
            ("Wall Straightness", detail.get("wall_straightness")),
            ("Wall Thickness", detail.get("wall_thickness")),
            ("Ghost Walls", detail.get("ghost_wall_score")),
            ("Symmetry", detail.get("symmetry_score")),
            ("Free Space", detail.get("free_space_consistency")),
            ("Occupied Density", detail.get("occupied_density_score")),
        ]

        bar_width = 25
        for label, val in metrics:
            if row >= max_y - 6:
                break
            if val is not None:
                filled = int(val * bar_width)
                empty = bar_width - filled
                bar = "\u2588" * filled + "\u2591" * empty
                line = f"  {label:<22} [{bar}] {val:.2f}"
                color = self._score_color(val)
            else:
                line = f"  {label:<22} [{'?':^{bar_width}}]  -- "
                color = 0
            self._addstr_safe(stdscr, row, 0, line, color)
            row += 1

        row += 1

        # Notes
        notes = detail.get("rating_notes")
        if notes:
            self._addstr_safe(stdscr, row, 0, "Notes:", curses.A_BOLD)
            row += 1
            self._addstr_safe(stdscr, row, 0, f"  {notes}")
            row += 1

        row += 1

        # Map preview
        self._addstr_safe(stdscr, row, 0, "Map Preview:", curses.A_BOLD)
        row += 1

        map_png = detail.get("map_png")
        if map_png and row < max_y - 3:
            ascii_lines = self._map_png_to_ascii(map_png, max_x - 4, max_y - row - 3)
            for line in ascii_lines:
                if row >= max_y - 2:
                    break
                self._addstr_safe(stdscr, row, 2, line)
                row += 1
        else:
            self._addstr_safe(stdscr, row, 2, "No map data")
            row += 1

        row += 1
        self._addstr_safe(
            stdscr,
            min(row, max_y - 1),
            0,
            "Keys: 1-5=rate, n=notes, Esc=back",
            curses.color_pair(3),
        )

    # ------------------------------------------------------------------
    # Drawing: compare mode
    # ------------------------------------------------------------------

    def _draw_compare(self, stdscr):
        max_y, max_x = stdscr.getmaxyx()
        row = 0

        run_a = self._runs[self._selected_run_idx]
        detail_a = self._get_detailed_run(run_a["id"])
        detail_b = self._get_detailed_run(self._compare_run_id)

        if detail_b is None:
            self._addstr_safe(stdscr, row, 0, "Compare run not found.")
            return

        run_num_a = run_a.get("run_number", "?")
        run_num_b = detail_b.get("run_number", "?")

        title = f"=== Compare: Run #{run_num_a} vs Run #{run_num_b} ==="
        self._addstr_safe(stdscr, row, 0, title, curses.color_pair(4) | curses.A_BOLD)
        row += 2

        col_label = 2
        col_a = 20
        col_b = 40

        self._addstr_safe(stdscr, row, col_label, "", curses.A_BOLD)
        self._addstr_safe(stdscr, row, col_a, f"Run #{run_num_a}", curses.A_BOLD)
        self._addstr_safe(stdscr, row, col_b, f"Run #{run_num_b}", curses.A_BOLD)
        row += 1

        sep = "-" * min(max_x - 4, 60)
        self._addstr_safe(stdscr, row, col_label, sep)
        row += 1

        metrics = [
            ("Composite", "composite_score"),
            ("Wall Str", "wall_straightness"),
            ("Thickness", "wall_thickness"),
            ("Ghost", "ghost_wall_score"),
            ("Symmetry", "symmetry_score"),
            ("Free Space", "free_space_consistency"),
            ("Density", "occupied_density_score"),
        ]

        bar_width = 8
        for label, key in metrics:
            if row >= max_y - 8:
                break

            val_a = detail_a.get(key)
            val_b = detail_b.get(key)

            self._addstr_safe(stdscr, row, col_label, f"{label}:", curses.A_BOLD)

            if val_a is not None:
                filled_a = int(val_a * bar_width)
                bar_a = "\u2588" * filled_a + "\u2591" * (bar_width - filled_a)
                text_a = f"{val_a:.2f} {bar_a}"
                color_a = self._score_color(val_a)
            else:
                text_a = "  --"
                color_a = 0

            if val_b is not None:
                filled_b = int(val_b * bar_width)
                bar_b = "\u2588" * filled_b + "\u2591" * (bar_width - filled_b)
                text_b = f"{val_b:.2f} {bar_b}"
                color_b = self._score_color(val_b)
            else:
                text_b = "  --"
                color_b = 0

            self._addstr_safe(stdscr, row, col_a, text_a, color_a)
            self._addstr_safe(stdscr, row, col_b, text_b, color_b)
            row += 1

        row += 1

        # Rating comparison
        rating_a = detail_a.get("rating")
        rating_b = detail_b.get("rating")
        self._addstr_safe(stdscr, row, col_label, "Rating:", curses.A_BOLD)
        self._addstr_safe(
            stdscr, row, col_a, f"{rating_a}/5" if rating_a is not None else "--"
        )
        self._addstr_safe(
            stdscr, row, col_b, f"{rating_b}/5" if rating_b is not None else "--"
        )
        row += 2

        # Parameter diffs
        self._addstr_safe(stdscr, row, 0, "Param diffs:", curses.A_BOLD)
        row += 1

        slam_a = self._parse_json_safe(detail_a.get("slam_params", "{}"))
        slam_b = self._parse_json_safe(detail_b.get("slam_params", "{}"))
        ekf_a = self._parse_json_safe(detail_a.get("ekf_params", "{}"))
        ekf_b = self._parse_json_safe(detail_b.get("ekf_params", "{}"))

        all_keys = sorted(
            set(
                list(slam_a.keys())
                + list(slam_b.keys())
                + list(ekf_a.keys())
                + list(ekf_b.keys())
            )
        )

        for key in all_keys:
            if row >= max_y - 2:
                break
            val_a_s = slam_a.get(key, ekf_a.get(key))
            val_b_s = slam_b.get(key, ekf_b.get(key))
            if val_a_s != val_b_s:
                a_str = str(val_a_s) if val_a_s is not None else "--"
                b_str = str(val_b_s) if val_b_s is not None else "--"
                self._addstr_safe(stdscr, row, col_label, f"{key}:")
                self._addstr_safe(stdscr, row, col_a, a_str)
                self._addstr_safe(stdscr, row, col_b, b_str)
                row += 1

        row += 1
        self._addstr_safe(
            stdscr, min(row, max_y - 1), 0, "Keys: Esc=back", curses.color_pair(3)
        )

    # ------------------------------------------------------------------
    # Actions
    # ------------------------------------------------------------------

    def _enter_runs_mode(self):
        """Transition from session list to run list for the selected session."""
        session = self._sessions[self._selected_session_idx]
        sid = session["id"]
        self._runs = self._session_runs.get(sid, [])
        self._selected_run_idx = 0
        self._compare_run_id = None
        self._mode = "runs"

    def _rate_run(self, rating):
        """Store a rating for the currently selected run."""
        if not self._runs:
            return
        run = self._runs[self._selected_run_idx]
        run_id = run["id"]

        # Preserve existing notes
        existing = self._get_detailed_run(run_id)
        notes = existing.get("rating_notes") if existing else None

        self._db.store_rating(run_id, rating, notes)

        # Invalidate cache
        self._detailed_runs.pop(run_id, None)

    def _add_notes(self, stdscr):
        """Prompt the user for notes text and store it."""
        if not self._runs:
            return
        run = self._runs[self._selected_run_idx]
        run_id = run["id"]

        text = self._input_text(stdscr, "Notes: ")
        if text is not None:
            existing = self._get_detailed_run(run_id)
            rating = existing.get("rating", 3) if existing else 3
            self._db.store_rating(run_id, rating, text)
            self._detailed_runs.pop(run_id, None)

    def _handle_compare_mark(self):
        """Mark a run for comparison or enter compare mode."""
        if not self._runs:
            return
        current_run = self._runs[self._selected_run_idx]
        current_id = current_run["id"]

        if self._compare_run_id is None:
            # First mark
            self._compare_run_id = current_id
        elif self._compare_run_id != current_id:
            # Second mark on a different run -- enter compare mode
            self._mode = "compare"
        else:
            # Same run pressed again -- deselect
            self._compare_run_id = None

    def _export_best_yaml(self, stdscr):
        """Export the best run's params as a merged YAML file."""
        if not self._sessions:
            return

        session = self._sessions[self._selected_session_idx]
        sid = session["id"]
        best_run = self._db.get_best_run(sid)
        if best_run is None:
            self._show_message(stdscr, "No completed runs to export.")
            return

        base_slam, base_ekf = self._get_baseline_params(session)
        run_slam = self._parse_json_safe(best_run.get("slam_params", "{}"))
        run_ekf = self._parse_json_safe(best_run.get("ekf_params", "{}"))

        merged = {
            "slam_toolbox": {
                "ros__parameters": {**base_slam, **run_slam},
            },
            "ekf_filter_node": {
                "ros__parameters": {**base_ekf, **run_ekf},
            },
        }

        filename = f"best_params_session_{sid}.yaml"
        filepath = os.path.join(os.getcwd(), filename)

        with open(filepath, "w") as f:
            yaml.dump(merged, f, default_flow_style=False, sort_keys=False)

        self._show_message(stdscr, f"Exported: {filepath}")

    def _export_report(self, stdscr):
        """Export a markdown report for the selected session."""
        if not self._sessions:
            return

        session = self._sessions[self._selected_session_idx]
        sid = session["id"]

        try:
            path = self._db.export_session_report(sid)
            self._show_message(stdscr, f"Report saved to: {path}")
        except Exception as e:
            self._show_message(stdscr, f"Export failed: {e}")

    # ------------------------------------------------------------------
    # Text input
    # ------------------------------------------------------------------

    def _input_text(self, stdscr, prompt):
        """Read a line of text from the user at the bottom of the screen.

        Returns the entered string, or None if the user pressed Escape.
        """
        max_y, max_x = stdscr.getmaxyx()
        input_row = max_y - 1
        curses.curs_set(1)
        stdscr.move(input_row, 0)
        stdscr.clrtoeol()
        self._addstr_safe(stdscr, input_row, 0, prompt, curses.A_BOLD)
        stdscr.refresh()

        text = []
        cursor_pos = len(prompt)

        while True:
            ch = stdscr.getch()
            if ch in (curses.KEY_ENTER, 10, 13):
                curses.curs_set(0)
                return "".join(text)
            elif ch == 27:  # Escape
                curses.curs_set(0)
                return None
            elif ch in (curses.KEY_BACKSPACE, 127, 8):
                if text:
                    text.pop()
                    cursor_pos -= 1
                    stdscr.move(input_row, cursor_pos)
                    stdscr.delch()
            elif 32 <= ch < 127:
                if cursor_pos < max_x - 1:
                    text.append(chr(ch))
                    self._addstr_safe(stdscr, input_row, cursor_pos, chr(ch))
                    cursor_pos += 1

            stdscr.refresh()

    # ------------------------------------------------------------------
    # Message display
    # ------------------------------------------------------------------

    def _show_message(self, stdscr, msg):
        """Show a temporary message at the bottom of the screen."""
        max_y, max_x = stdscr.getmaxyx()
        row = max_y - 1
        stdscr.move(row, 0)
        stdscr.clrtoeol()
        self._addstr_safe(stdscr, row, 0, msg, curses.color_pair(1) | curses.A_BOLD)
        stdscr.refresh()
        stdscr.getch()  # Wait for any key

    # ------------------------------------------------------------------
    # Data helpers
    # ------------------------------------------------------------------

    def _get_detailed_run(self, run_id):
        """Fetch and cache full run details including analysis and rating."""
        if run_id not in self._detailed_runs:
            detail = self._db.get_run(run_id)
            if detail is not None:
                self._detailed_runs[run_id] = detail
            else:
                return {}
        return self._detailed_runs[run_id]

    def _get_baseline_params(self, session):
        """Parse the baseline SLAM and EKF params from a session dict."""
        base_slam = self._parse_json_safe(session.get("base_slam_config", "{}"))
        base_ekf = self._parse_json_safe(session.get("base_ekf_config", "{}"))
        return base_slam, base_ekf

    def _get_changed_params_str(self, detail, base_slam, base_ekf):
        """Return a compact string describing which params changed from baseline."""
        run_slam = self._parse_json_safe(detail.get("slam_params", "{}"))
        run_ekf = self._parse_json_safe(detail.get("ekf_params", "{}"))

        changed = []
        for key, value in run_slam.items():
            if base_slam.get(key) != value:
                changed.append(f"{key}={value}")
        for key, value in run_ekf.items():
            if base_ekf.get(key) != value:
                changed.append(f"{key}={value}")

        return ", ".join(changed) if changed else "baseline"

    @staticmethod
    def _parse_json_safe(text):
        """Parse a JSON string, returning an empty dict on failure."""
        if not text:
            return {}
        try:
            return json.loads(text)
        except (json.JSONDecodeError, TypeError):
            return {}

    # ------------------------------------------------------------------
    # Scoring helpers
    # ------------------------------------------------------------------

    def _score_color(self, val):
        """Return a curses color pair based on the score value."""
        if val is None:
            return 0
        if val >= 0.8:
            return curses.color_pair(1)  # Green
        if val >= 0.6:
            return curses.color_pair(3)  # Yellow
        return curses.color_pair(2)  # Red

    # ------------------------------------------------------------------
    # Map PNG to ASCII
    # ------------------------------------------------------------------

    def _map_png_to_ascii(self, png_bytes, max_width, max_height):
        """Convert a PNG image (as bytes) to a list of ASCII art lines.

        Uses brightness-to-character mapping. The PNG is resized to fit
        within the given dimensions.
        """
        try:
            import numpy as np

            nparr = np.frombuffer(png_bytes, np.uint8)
            import cv2

            img = cv2.imdecode(nparr, cv2.IMREAD_GRAYSCALE)
            if img is None:
                return ["(failed to decode map image)"]
        except ImportError:
            return ["(numpy/cv2 not available for map preview)"]
        except Exception:
            return ["(failed to decode map image)"]

        h, w = img.shape

        # Target dimensions -- ASCII chars are roughly 2x taller than wide
        target_w = min(max_width, 60)
        target_h = min(max_height, 30)

        # Scale maintaining aspect ratio, accounting for character aspect
        scale_w = target_w / w
        scale_h = (target_h * 2.0) / h  # *2 because chars are ~2:1 tall
        scale = min(scale_w, scale_h)

        new_w = max(1, int(w * scale))
        new_h = max(1, int(h * scale / 2.0))  # /2 to account for char height

        resized = cv2.resize(img, (new_w, new_h), interpolation=cv2.INTER_AREA)

        lines = []
        num_chars = len(_ASCII_CHARS)
        for row_idx in range(new_h):
            line_chars = []
            for col_idx in range(new_w):
                brightness = resized[row_idx, col_idx]
                # Map 0-255 to character index (0=darkest, 255=lightest)
                char_idx = int(brightness / 255.0 * (num_chars - 1))
                char_idx = min(char_idx, num_chars - 1)
                line_chars.append(_ASCII_CHARS[char_idx])
            lines.append("".join(line_chars))

        return lines

    # ------------------------------------------------------------------
    # Safe curses output
    # ------------------------------------------------------------------

    @staticmethod
    def _addstr_safe(stdscr, row, col, text, attr=0):
        """Write text to stdscr, silently truncating if it goes out of bounds."""
        max_y, max_x = stdscr.getmaxyx()
        if row < 0 or row >= max_y or col >= max_x:
            return
        # Truncate text to fit within the window width
        available = max_x - col - 1
        if available <= 0:
            return
        truncated = text[:available]
        try:
            stdscr.addstr(row, col, truncated, attr)
        except curses.error:
            pass


# ------------------------------------------------------------------
# Entry point
# ------------------------------------------------------------------


def main(args=None):
    import sys

    db_path = os.path.expanduser("~/.local/share/mapping_autotune/autotune.db")

    # Check for --db-path command-line argument
    for i, arg in enumerate(sys.argv):
        if arg == "--db-path" and i + 1 < len(sys.argv):
            db_path = sys.argv[i + 1]

    # Try to use ROS parameters if available
    try:
        rclpy.init(args=args)
        from rclpy.node import Node

        node = Node("review_tui_node")
        node.declare_parameter("db_path", db_path)
        db_path = node.get_parameter("db_path").value

        node.declare_parameter("export_report", False)
        export_only = node.get_parameter("export_report").value

        if export_only:
            node.declare_parameter("session_id", 0)
            sid = node.get_parameter("session_id").value
            db = DbManager(db_path)
            path = db.export_session_report(sid)
            print(f"Report saved to: {path}")
            node.destroy_node()
            rclpy.shutdown()
            return

        node.destroy_node()
        rclpy.shutdown()
    except Exception:
        pass

    tui = ReviewTui(db_path)
    tui.run()


if __name__ == "__main__":
    main()
