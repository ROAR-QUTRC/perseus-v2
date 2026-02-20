#!/usr/bin/env python3
"""Standalone review TUI for mapping autotune — no ROS dependency.

Run on any machine with Python 3, sqlite3, and a terminal:
    python3 review_tui_standalone.py --db-path /srv/mapping_autotune/autotune.db
"""
import argparse
import curses
import json
import os
import sqlite3
import sys


class StandaloneReviewTui:
    def __init__(self, db_path):
        self._db_path = db_path
        self._conn = sqlite3.connect(db_path)
        self._conn.row_factory = sqlite3.Row
        self._mode = "sessions"
        self._sessions = []
        self._runs = []
        self._sel_session = 0
        self._sel_run = 0
        self._scroll_offset = 0
        self._message = ""

    def run(self):
        curses.wrapper(self._main_loop)

    def _load_sessions(self):
        cur = self._conn.execute(
            "SELECT s.*, "
            "(SELECT COUNT(*) FROM runs r WHERE r.session_id=s.id) as run_count, "
            "(SELECT MAX(a.composite_score) FROM analysis a "
            " JOIN runs r ON a.run_id=r.id WHERE r.session_id=s.id) as best_score "
            "FROM sessions s ORDER BY s.id DESC"
        )
        self._sessions = [dict(row) for row in cur.fetchall()]

    def _load_runs(self, session_id):
        cur = self._conn.execute(
            "SELECT r.*, a.composite_score, a.wall_straightness, a.wall_thickness, "
            "a.ghost_wall_score, a.symmetry_score, a.free_space_consistency, "
            "a.occupied_density_score, rat.rating, rat.notes as rating_notes "
            "FROM runs r "
            "LEFT JOIN analysis a ON a.run_id=r.id "
            "LEFT JOIN ratings rat ON rat.run_id=r.id "
            "WHERE r.session_id=? ORDER BY r.run_number",
            (session_id,),
        )
        self._runs = [dict(row) for row in cur.fetchall()]

    def _main_loop(self, stdscr):
        curses.start_color()
        curses.use_default_colors()
        curses.init_pair(1, curses.COLOR_GREEN, -1)
        curses.init_pair(2, curses.COLOR_RED, -1)
        curses.init_pair(3, curses.COLOR_YELLOW, -1)
        curses.init_pair(4, curses.COLOR_CYAN, -1)
        curses.init_pair(5, curses.COLOR_BLACK, curses.COLOR_WHITE)
        curses.curs_set(0)

        self._load_sessions()

        while True:
            stdscr.erase()
            h, w = stdscr.getmaxyx()

            if self._mode == "sessions":
                self._draw_sessions(stdscr, h, w)
            elif self._mode == "runs":
                self._draw_runs(stdscr, h, w)
            elif self._mode == "detail":
                self._draw_detail(stdscr, h, w)

            if self._message:
                stdscr.addnstr(h - 1, 0, self._message, w - 1, curses.color_pair(3))

            stdscr.refresh()
            key = stdscr.getch()
            self._message = ""

            if key == ord("q"):
                break
            self._handle_key(key)

    def _score_color(self, score):
        if score is None:
            return 0
        if score >= 0.8:
            return curses.color_pair(1)
        if score >= 0.6:
            return curses.color_pair(3)
        return curses.color_pair(2)

    def _draw_sessions(self, stdscr, h, w):
        stdscr.addnstr(0, 0, "=== MAPPING AUTOTUNE REVIEW ===", w - 1, curses.color_pair(4))
        header = f"{'#':>3} | {'Name':<20} | {'Date':<10} | {'Runs':>4} | {'Best':>6} | Status"
        stdscr.addnstr(2, 0, header, w - 1, curses.color_pair(4) | curses.A_BOLD)

        for i, s in enumerate(self._sessions):
            if i + 3 >= h - 2:
                break
            score_str = f"{s['best_score']:.2f}" if s["best_score"] else "  --  "
            line = (
                f"{s['id']:>3} | {(s['name'] or ''):<20} | "
                f"{(s['created_at'] or '')[:10]:<10} | {s['run_count']:>4} | "
                f"{score_str:>6} | {s['status'] or ''}"
            )
            attr = curses.color_pair(5) if i == self._sel_session else 0
            prefix = "> " if i == self._sel_session else "  "
            stdscr.addnstr(i + 3, 0, prefix + line, w - 1, attr)

        keys = "Keys: Enter=view runs, r=export report, q=quit"
        stdscr.addnstr(h - 2, 0, keys, w - 1)

    def _draw_runs(self, stdscr, h, w):
        if not self._sessions:
            return
        sid = self._sessions[self._sel_session]["id"]
        sname = self._sessions[self._sel_session]["name"] or f"Session {sid}"
        stdscr.addnstr(0, 0, f"=== {sname} ({len(self._runs)} runs) ===", w - 1,
                        curses.color_pair(4))

        header = (
            f"{'#':>3} | {'Score':>6} | {'Wall':>5} | {'Thick':>5} | "
            f"{'Ghost':>5} | {'Symm':>5} | {'Free':>5} | {'Dens':>5} | "
            f"{'Rate':>4} | Params"
        )
        stdscr.addnstr(2, 0, header[:w - 1], w - 1, curses.color_pair(4) | curses.A_BOLD)

        max_rows = h - 5
        for i, r in enumerate(self._runs):
            if i >= max_rows:
                break
            cs = r.get("composite_score")
            score_str = f"{cs:.2f}" if cs is not None else "  -- "

            def fmt(v):
                return f"{v:.2f}" if v is not None else " -- "

            rating_str = f"{r['rating']}/5" if r.get("rating") else " -- "

            # Extract key changed params
            try:
                slam_p = json.loads(r.get("slam_params", "{}"))
                params_str = ", ".join(f"{k}={v}" for k, v in list(slam_p.items())[:2])
                if not params_str:
                    params_str = "baseline"
            except (json.JSONDecodeError, TypeError):
                params_str = "baseline"

            line = (
                f"{r['run_number']:>3} | {score_str:>6} | "
                f"{fmt(r.get('wall_straightness')):>5} | "
                f"{fmt(r.get('wall_thickness')):>5} | "
                f"{fmt(r.get('ghost_wall_score')):>5} | "
                f"{fmt(r.get('symmetry_score')):>5} | "
                f"{fmt(r.get('free_space_consistency')):>5} | "
                f"{fmt(r.get('occupied_density_score')):>5} | "
                f"{rating_str:>4} | {params_str}"
            )

            attr = curses.color_pair(5) if i == self._sel_run else 0
            prefix = "> " if i == self._sel_run else "  "
            stdscr.addnstr(i + 3, 0, (prefix + line)[:w - 1], w - 1, attr)

        keys = "Keys: Enter=detail, 1-5=rate, n=notes, e=export YAML, Esc=back, q=quit"
        stdscr.addnstr(h - 2, 0, keys, w - 1)

    def _draw_detail(self, stdscr, h, w):
        if not self._runs or self._sel_run >= len(self._runs):
            return
        r = self._runs[self._sel_run]
        cs = r.get("composite_score")
        rating_str = f"{r['rating']}/5" if r.get("rating") else "--"

        stdscr.addnstr(0, 0, f"=== Run #{r['run_number']} Detail ===", w - 1,
                        curses.color_pair(4))
        stdscr.addnstr(1, 0,
                        f"Status: {r['status']} | Composite: {cs:.2f if cs else '--'} | "
                        f"Rating: {rating_str}", w - 1)

        row = 3
        stdscr.addnstr(row, 0, "Parameters:", w - 1, curses.A_BOLD)
        row += 1
        try:
            params = json.loads(r.get("slam_params", "{}"))
            for k, v in params.items():
                if row >= h - 6:
                    break
                stdscr.addnstr(row, 2, f"{k}: {v}", w - 3)
                row += 1
        except (json.JSONDecodeError, TypeError):
            pass

        row += 1
        stdscr.addnstr(row, 0, "Metrics:", w - 1, curses.A_BOLD)
        row += 1
        metrics = [
            ("Wall Straightness", r.get("wall_straightness")),
            ("Wall Thickness", r.get("wall_thickness")),
            ("Ghost Walls", r.get("ghost_wall_score")),
            ("Symmetry", r.get("symmetry_score")),
            ("Free Space", r.get("free_space_consistency")),
            ("Occupied Density", r.get("occupied_density_score")),
        ]
        bar_width = 25
        for name, val in metrics:
            if row >= h - 3:
                break
            if val is not None:
                filled = int(val * bar_width)
                bar = "\u2588" * filled + "\u2591" * (bar_width - filled)
                line = f"  {name:<22} [{bar}] {val:.2f}"
                stdscr.addnstr(row, 0, line[:w - 1], w - 1, self._score_color(val))
            else:
                stdscr.addnstr(row, 0, f"  {name:<22} [no data]", w - 1)
            row += 1

        if r.get("rating_notes"):
            row += 1
            stdscr.addnstr(row, 0, f"Notes: {r['rating_notes']}", w - 1)

        keys = "Keys: 1-5=rate, n=notes, Esc=back"
        stdscr.addnstr(h - 2, 0, keys, w - 1)

    def _handle_key(self, key):
        if self._mode == "sessions":
            if key == curses.KEY_UP and self._sel_session > 0:
                self._sel_session -= 1
            elif key == curses.KEY_DOWN and self._sel_session < len(self._sessions) - 1:
                self._sel_session += 1
            elif key in (curses.KEY_ENTER, 10, 13) and self._sessions:
                sid = self._sessions[self._sel_session]["id"]
                self._load_runs(sid)
                self._sel_run = 0
                self._mode = "runs"
            elif key == ord("r") and self._sessions:
                self._message = "Report export not available in standalone mode"

        elif self._mode == "runs":
            if key == curses.KEY_UP and self._sel_run > 0:
                self._sel_run -= 1
            elif key == curses.KEY_DOWN and self._sel_run < len(self._runs) - 1:
                self._sel_run += 1
            elif key in (curses.KEY_ENTER, 10, 13) and self._runs:
                self._mode = "detail"
            elif key == 27:  # Esc
                self._mode = "sessions"
            elif key in range(ord("1"), ord("6")) and self._runs:
                rating = key - ord("0")
                run_id = self._runs[self._sel_run]["id"]
                self._conn.execute(
                    "INSERT OR REPLACE INTO ratings (run_id, rating) VALUES (?, ?)",
                    (run_id, rating),
                )
                self._conn.commit()
                self._runs[self._sel_run]["rating"] = rating
                self._message = f"Rated run #{self._runs[self._sel_run]['run_number']}: {rating}/5"

        elif self._mode == "detail":
            if key == 27:  # Esc
                self._mode = "runs"
            elif key in range(ord("1"), ord("6")) and self._runs:
                rating = key - ord("0")
                run_id = self._runs[self._sel_run]["id"]
                self._conn.execute(
                    "INSERT OR REPLACE INTO ratings (run_id, rating) VALUES (?, ?)",
                    (run_id, rating),
                )
                self._conn.commit()
                self._runs[self._sel_run]["rating"] = rating
                self._message = f"Rated: {rating}/5"

    def __del__(self):
        if hasattr(self, "_conn"):
            self._conn.close()


def main():
    parser = argparse.ArgumentParser(description="Review mapping autotune results")
    parser.add_argument("--db-path", default="/opt/mapping_autotune/autotune.db",
                        help="Path to the SQLite database")
    args = parser.parse_args()

    if not os.path.exists(args.db_path):
        print(f"Database not found: {args.db_path}")
        print("Run setup_mapping_db.sh first.")
        sys.exit(1)

    tui = StandaloneReviewTui(args.db_path)
    tui.run()


if __name__ == "__main__":
    main()
