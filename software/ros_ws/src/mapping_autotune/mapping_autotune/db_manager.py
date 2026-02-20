import json
import sqlite3
import zlib
import subprocess
import os
import datetime


class DbManager:
    """SQLite database manager for the mapping autotune system.

    Handles CRUD operations on sessions, runs, analysis results, and human
    ratings.  Supports optional rsync-based remote sync and markdown report
    export.
    """

    def __init__(self, db_path="/opt/mapping_autotune/autotune.db", logger=None):
        self._db_path = db_path
        self._logger = logger
        self._sync_config = None

        self.init_db()

        sync_config_path = os.path.join(os.path.dirname(self._db_path), "sync_config.json")
        if os.path.isfile(sync_config_path):
            try:
                with open(sync_config_path, "r") as f:
                    self._sync_config = json.load(f)
                self._log_info(f"Loaded sync config from {sync_config_path}")
            except Exception as e:
                self._log_warn(f"Failed to load sync config: {e}")

    # ------------------------------------------------------------------
    # Logging helpers
    # ------------------------------------------------------------------

    def _log_info(self, msg):
        if self._logger is not None:
            self._logger.info(msg)

    def _log_warn(self, msg):
        if self._logger is not None:
            self._logger.warn(msg)

    def _log_error(self, msg):
        if self._logger is not None:
            self._logger.error(msg)

    # ------------------------------------------------------------------
    # Connection helpers
    # ------------------------------------------------------------------

    def _connect(self):
        conn = sqlite3.connect(self._db_path)
        conn.row_factory = sqlite3.Row
        conn.execute("PRAGMA foreign_keys = ON")
        return conn

    # ------------------------------------------------------------------
    # Schema
    # ------------------------------------------------------------------

    def init_db(self):
        """Create all tables if they do not already exist."""
        os.makedirs(os.path.dirname(self._db_path), exist_ok=True)
        conn = self._connect()
        try:
            conn.executescript(
                """
                CREATE TABLE IF NOT EXISTS sessions (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    name TEXT NOT NULL,
                    created_at TEXT DEFAULT (datetime('now')),
                    description TEXT,
                    base_slam_config TEXT NOT NULL,
                    base_ekf_config TEXT NOT NULL,
                    status TEXT DEFAULT 'active'
                );

                CREATE TABLE IF NOT EXISTS runs (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    session_id INTEGER NOT NULL REFERENCES sessions(id),
                    run_number INTEGER NOT NULL,
                    started_at TEXT DEFAULT (datetime('now')),
                    completed_at TEXT,
                    status TEXT DEFAULT 'pending',
                    slam_params TEXT NOT NULL,
                    ekf_params TEXT NOT NULL,
                    imu_params TEXT DEFAULT '{}',
                    maneuver_params TEXT DEFAULT '{}',
                    map_data BLOB,
                    map_width INTEGER,
                    map_height INTEGER,
                    map_resolution REAL,
                    map_png BLOB,
                    odom_log TEXT,
                    imu_log TEXT,
                    UNIQUE(session_id, run_number)
                );

                CREATE TABLE IF NOT EXISTS analysis (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    run_id INTEGER NOT NULL UNIQUE REFERENCES runs(id),
                    wall_straightness REAL,
                    wall_thickness REAL,
                    ghost_wall_score REAL,
                    symmetry_score REAL,
                    free_space_consistency REAL,
                    occupied_density_score REAL,
                    composite_score REAL NOT NULL,
                    diagnostics TEXT
                );

                CREATE TABLE IF NOT EXISTS ratings (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    run_id INTEGER NOT NULL REFERENCES runs(id),
                    rated_at TEXT DEFAULT (datetime('now')),
                    rating INTEGER NOT NULL CHECK (rating BETWEEN 1 AND 5),
                    notes TEXT,
                    UNIQUE(run_id)
                );
                """
            )
            conn.commit()
            self._log_info("Database initialised successfully")
        finally:
            conn.close()

    # ------------------------------------------------------------------
    # Health check
    # ------------------------------------------------------------------

    def check_connection(self):
        """Return True if the database is reachable, False otherwise."""
        try:
            conn = self._connect()
            conn.execute("SELECT 1")
            conn.close()
            return True
        except Exception:
            return False

    # ------------------------------------------------------------------
    # Session CRUD
    # ------------------------------------------------------------------

    def create_session(self, name, description, base_slam_config, base_ekf_config):
        """Create a new tuning session and return its id."""
        conn = self._connect()
        try:
            cursor = conn.execute(
                "INSERT INTO sessions (name, description, base_slam_config, base_ekf_config) "
                "VALUES (?, ?, ?, ?)",
                (name, description, base_slam_config, base_ekf_config),
            )
            conn.commit()
            session_id = cursor.lastrowid
            self._log_info(f"Created session {session_id}: {name}")
            return session_id
        finally:
            conn.close()

    def update_session_status(self, session_id, status):
        """Update the status field of a session."""
        conn = self._connect()
        try:
            conn.execute(
                "UPDATE sessions SET status = ? WHERE id = ?",
                (status, session_id),
            )
            conn.commit()
        finally:
            conn.close()

    def get_session(self, session_id):
        """Return a single session as a dict, or None."""
        conn = self._connect()
        try:
            row = conn.execute(
                "SELECT * FROM sessions WHERE id = ?", (session_id,)
            ).fetchone()
            return dict(row) if row else None
        finally:
            conn.close()

    def get_all_sessions(self):
        """Return every session as a list of dicts."""
        conn = self._connect()
        try:
            rows = conn.execute(
                "SELECT * FROM sessions ORDER BY created_at DESC"
            ).fetchall()
            return [dict(r) for r in rows]
        finally:
            conn.close()

    # ------------------------------------------------------------------
    # Run CRUD
    # ------------------------------------------------------------------

    def create_run(self, session_id, run_number, slam_params, ekf_params,
                   imu_params="{}", maneuver_params="{}"):
        """Create a new run record and return its id."""
        conn = self._connect()
        try:
            cursor = conn.execute(
                "INSERT INTO runs (session_id, run_number, slam_params, ekf_params, "
                "imu_params, maneuver_params) VALUES (?, ?, ?, ?, ?, ?)",
                (session_id, run_number, slam_params, ekf_params,
                 imu_params, maneuver_params),
            )
            conn.commit()
            run_id = cursor.lastrowid
            self._log_info(f"Created run {run_id} (session {session_id}, #{run_number})")
            return run_id
        finally:
            conn.close()

    def update_run_status(self, run_id, status, completed_at=None):
        """Update a run's status and optionally its completion timestamp."""
        conn = self._connect()
        try:
            if completed_at is not None:
                conn.execute(
                    "UPDATE runs SET status = ?, completed_at = ? WHERE id = ?",
                    (status, completed_at, run_id),
                )
            else:
                conn.execute(
                    "UPDATE runs SET status = ? WHERE id = ?",
                    (status, run_id),
                )
            conn.commit()
        finally:
            conn.close()

    def store_map_data(self, run_id, map_data_compressed, width, height, resolution,
                       map_png=None):
        """Store pre-compressed map occupancy grid and optional PNG thumbnail."""
        conn = self._connect()
        try:
            conn.execute(
                "UPDATE runs SET map_data = ?, map_width = ?, map_height = ?, "
                "map_resolution = ?, map_png = ? WHERE id = ?",
                (map_data_compressed, width, height, resolution, map_png, run_id),
            )
            conn.commit()
            self._log_info(
                f"Stored map data for run {run_id} ({width}x{height}, "
                f"res={resolution})"
            )
        finally:
            conn.close()

    def store_sensor_logs(self, run_id, odom_log, imu_log):
        """Store odometry and IMU log JSON strings for a run."""
        conn = self._connect()
        try:
            conn.execute(
                "UPDATE runs SET odom_log = ?, imu_log = ? WHERE id = ?",
                (odom_log, imu_log, run_id),
            )
            conn.commit()
        finally:
            conn.close()

    def get_runs(self, session_id):
        """Return all runs for a session as a list of dicts (no BLOBs)."""
        conn = self._connect()
        try:
            rows = conn.execute(
                "SELECT id, session_id, run_number, started_at, completed_at, "
                "status, slam_params, ekf_params, imu_params, maneuver_params, "
                "map_width, map_height, map_resolution "
                "FROM runs WHERE session_id = ? ORDER BY run_number",
                (session_id,),
            ).fetchall()
            return [dict(r) for r in rows]
        finally:
            conn.close()

    def get_run(self, run_id):
        """Return a single run joined with its analysis and rating."""
        conn = self._connect()
        try:
            row = conn.execute(
                "SELECT r.*, "
                "a.wall_straightness, a.wall_thickness, a.ghost_wall_score, "
                "a.symmetry_score, a.free_space_consistency, "
                "a.occupied_density_score, a.composite_score, a.diagnostics, "
                "rt.rating, rt.notes AS rating_notes, rt.rated_at "
                "FROM runs r "
                "LEFT JOIN analysis a ON a.run_id = r.id "
                "LEFT JOIN ratings rt ON rt.run_id = r.id "
                "WHERE r.id = ?",
                (run_id,),
            ).fetchone()
            return dict(row) if row else None
        finally:
            conn.close()

    def get_best_run(self, session_id):
        """Return the run with the highest composite score for a session."""
        conn = self._connect()
        try:
            row = conn.execute(
                "SELECT r.*, "
                "a.wall_straightness, a.wall_thickness, a.ghost_wall_score, "
                "a.symmetry_score, a.free_space_consistency, "
                "a.occupied_density_score, a.composite_score, a.diagnostics, "
                "rt.rating, rt.notes AS rating_notes, rt.rated_at "
                "FROM runs r "
                "JOIN analysis a ON a.run_id = r.id "
                "LEFT JOIN ratings rt ON rt.run_id = r.id "
                "WHERE r.session_id = ? "
                "ORDER BY a.composite_score DESC LIMIT 1",
                (session_id,),
            ).fetchone()
            return dict(row) if row else None
        finally:
            conn.close()

    # ------------------------------------------------------------------
    # Analysis
    # ------------------------------------------------------------------

    def store_analysis(self, run_id, metrics_dict):
        """Insert or replace analysis metrics for a run.

        Expected keys in *metrics_dict*: wall_straightness, wall_thickness,
        ghost_wall_score, symmetry_score, free_space_consistency,
        occupied_density_score, composite_score, diagnostics.
        """
        conn = self._connect()
        try:
            conn.execute(
                "INSERT OR REPLACE INTO analysis "
                "(run_id, wall_straightness, wall_thickness, ghost_wall_score, "
                "symmetry_score, free_space_consistency, occupied_density_score, "
                "composite_score, diagnostics) "
                "VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)",
                (
                    run_id,
                    metrics_dict.get("wall_straightness"),
                    metrics_dict.get("wall_thickness"),
                    metrics_dict.get("ghost_wall_score"),
                    metrics_dict.get("symmetry_score"),
                    metrics_dict.get("free_space_consistency"),
                    metrics_dict.get("occupied_density_score"),
                    metrics_dict["composite_score"],
                    metrics_dict.get("diagnostics"),
                ),
            )
            conn.commit()
            self._log_info(
                f"Stored analysis for run {run_id} "
                f"(composite={metrics_dict['composite_score']:.4f})"
            )
        finally:
            conn.close()

    # ------------------------------------------------------------------
    # Ratings
    # ------------------------------------------------------------------

    def store_rating(self, run_id, rating, notes=None):
        """Store or update a human rating (1-5) for a run."""
        conn = self._connect()
        try:
            conn.execute(
                "INSERT OR REPLACE INTO ratings (run_id, rating, notes) "
                "VALUES (?, ?, ?)",
                (run_id, rating, notes),
            )
            conn.commit()
            self._log_info(f"Stored rating {rating}/5 for run {run_id}")
        finally:
            conn.close()

    # ------------------------------------------------------------------
    # Remote sync
    # ------------------------------------------------------------------

    def sync_to_remote(self):
        """Rsync the database file to a remote host if sync_config is loaded.

        The sync config JSON is expected to contain at minimum:
            remote_host, remote_path
        and optionally:
            ssh_key, rsync_flags
        """
        if self._sync_config is None:
            self._log_warn("No sync config loaded, skipping remote sync")
            return

        remote_host = self._sync_config.get("remote_host")
        remote_path = self._sync_config.get("remote_path")
        if not remote_host or not remote_path:
            self._log_warn("Incomplete sync config (need remote_host and remote_path)")
            return

        cmd = ["rsync", "-az"]

        extra_flags = self._sync_config.get("rsync_flags")
        if extra_flags:
            if isinstance(extra_flags, list):
                cmd.extend(extra_flags)
            else:
                cmd.append(extra_flags)

        ssh_key = self._sync_config.get("ssh_key")
        if ssh_key:
            cmd.extend(["-e", f"ssh -i {ssh_key}"])

        cmd.append(self._db_path)
        cmd.append(f"{remote_host}:{remote_path}")

        self._log_info(f"Syncing database to {remote_host}:{remote_path}")
        try:
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
            if result.returncode != 0:
                self._log_warn(f"rsync failed (rc={result.returncode}): {result.stderr}")
            else:
                self._log_info("Remote sync completed successfully")
        except subprocess.TimeoutExpired:
            self._log_warn("rsync timed out after 30 seconds")
        except Exception as e:
            self._log_warn(f"Remote sync failed: {e}")

    # ------------------------------------------------------------------
    # Report export
    # ------------------------------------------------------------------

    def export_session_report(self, session_id, output_dir=None):
        """Generate a comprehensive markdown report for a tuning session.

        Returns the file path of the written report.
        """
        session = self.get_session(session_id)
        if session is None:
            raise ValueError(f"Session {session_id} not found")

        runs = self.get_runs(session_id)
        best_run = self.get_best_run(session_id)

        # Gather full run details including analysis and ratings
        detailed_runs = []
        for run in runs:
            detailed = self.get_run(run["id"])
            if detailed:
                detailed_runs.append(detailed)

        completed_count = sum(1 for r in runs if r["status"] == "completed")
        failed_count = sum(1 for r in runs if r["status"] == "failed")

        # Parse baseline configs
        try:
            base_slam = json.loads(session["base_slam_config"])
        except (json.JSONDecodeError, TypeError):
            base_slam = {}
        try:
            base_ekf = json.loads(session["base_ekf_config"])
        except (json.JSONDecodeError, TypeError):
            base_ekf = {}

        lines = []

        # --- Header ---
        lines.append(f"# Mapping Autotune Report — Session: {session['name']}")
        lines.append(f"Date: {session['created_at']}")
        lines.append("Robot: Perseus Lite")
        lines.append(
            f"Total Runs: {len(runs)} | Completed: {completed_count} | "
            f"Failed: {failed_count}"
        )
        lines.append("")

        # --- Best Run ---
        if best_run:
            composite = best_run.get("composite_score", 0.0)
            lines.append(
                f"## Best Run: #{best_run['run_number']} "
                f"(Composite Score: {composite:.2f})"
            )
            lines.append("")

            # Parameter diff from baseline
            lines.append("### Parameters (diff from baseline)")
            try:
                run_slam = json.loads(best_run.get("slam_params", "{}"))
            except (json.JSONDecodeError, TypeError):
                run_slam = {}
            try:
                run_ekf = json.loads(best_run.get("ekf_params", "{}"))
            except (json.JSONDecodeError, TypeError):
                run_ekf = {}

            diff_found = False
            for key, value in run_slam.items():
                baseline_val = base_slam.get(key)
                if baseline_val != value:
                    lines.append(f"- {key}: {value} (baseline: {baseline_val})")
                    diff_found = True
            for key, value in run_ekf.items():
                baseline_val = base_ekf.get(key)
                if baseline_val != value:
                    lines.append(f"- {key}: {value} (baseline: {baseline_val})")
                    diff_found = True
            if not diff_found:
                lines.append("- No parameter changes from baseline")
            lines.append("")

            # Analysis scores
            lines.append("### Analysis Scores")
            lines.append("| Metric | Score |")
            lines.append("|--------|-------|")
            metric_names = [
                ("Wall Straightness", "wall_straightness"),
                ("Wall Thickness", "wall_thickness"),
                ("Ghost Wall Score", "ghost_wall_score"),
                ("Symmetry Score", "symmetry_score"),
                ("Free Space Consistency", "free_space_consistency"),
                ("Occupied Density Score", "occupied_density_score"),
                ("Composite Score", "composite_score"),
            ]
            for label, key in metric_names:
                val = best_run.get(key)
                if val is not None:
                    lines.append(f"| {label} | {val:.2f} |")
                else:
                    lines.append(f"| {label} | N/A |")
            lines.append("")

        # --- All Runs Summary ---
        lines.append("## All Runs Summary")
        lines.append(
            "| Run | Composite | Wall Str. | Thickness | Ghosts | "
            "Symmetry | Free Space | Density | Rating | Key Params Changed |"
        )
        lines.append(
            "|-----|-----------|-----------|-----------|--------|"
            "----------|------------|---------|--------|-------------------|"
        )
        for dr in detailed_runs:
            run_num = dr.get("run_number", "?")

            def _fmt(val):
                return f"{val:.2f}" if val is not None else "N/A"

            # Determine key params changed
            try:
                run_slam = json.loads(dr.get("slam_params", "{}"))
            except (json.JSONDecodeError, TypeError):
                run_slam = {}
            try:
                run_ekf = json.loads(dr.get("ekf_params", "{}"))
            except (json.JSONDecodeError, TypeError):
                run_ekf = {}

            changed_params = []
            for key, value in run_slam.items():
                if base_slam.get(key) != value:
                    changed_params.append(f"{key}={value}")
            for key, value in run_ekf.items():
                if base_ekf.get(key) != value:
                    changed_params.append(f"{key}={value}")
            changed_str = ", ".join(changed_params) if changed_params else "baseline"

            rating_val = dr.get("rating")
            rating_str = f"{rating_val}/5" if rating_val is not None else "-"

            lines.append(
                f"| {run_num} "
                f"| {_fmt(dr.get('composite_score'))} "
                f"| {_fmt(dr.get('wall_straightness'))} "
                f"| {_fmt(dr.get('wall_thickness'))} "
                f"| {_fmt(dr.get('ghost_wall_score'))} "
                f"| {_fmt(dr.get('symmetry_score'))} "
                f"| {_fmt(dr.get('free_space_consistency'))} "
                f"| {_fmt(dr.get('occupied_density_score'))} "
                f"| {rating_str} "
                f"| {changed_str} |"
            )
        lines.append("")

        # --- Parameter Trends ---
        lines.append("## Parameter Trends")
        lines.append("")

        # Collect all varied parameters and their composite scores
        param_scores = {}
        for dr in detailed_runs:
            composite = dr.get("composite_score")
            if composite is None:
                continue
            try:
                run_slam = json.loads(dr.get("slam_params", "{}"))
            except (json.JSONDecodeError, TypeError):
                run_slam = {}
            try:
                run_ekf = json.loads(dr.get("ekf_params", "{}"))
            except (json.JSONDecodeError, TypeError):
                run_ekf = {}
            all_params = {**run_slam, **run_ekf}
            for key, value in all_params.items():
                baseline_val = base_slam.get(key, base_ekf.get(key))
                if baseline_val != value:
                    if key not in param_scores:
                        param_scores[key] = []
                    param_scores[key].append((value, composite))

        if param_scores:
            for param_name, entries in sorted(param_scores.items()):
                lines.append(f"### {param_name}")
                lines.append("| Value | Composite Score |")
                lines.append("|-------|-----------------|")
                for value, score in sorted(entries, key=lambda x: x[1], reverse=True):
                    lines.append(f"| {value} | {score:.2f} |")
                lines.append("")
        else:
            lines.append("No parameter variations recorded.")
            lines.append("")

        # --- Human Ratings Summary ---
        lines.append("## Human Ratings Summary")
        lines.append("")
        rated_runs = [dr for dr in detailed_runs if dr.get("rating") is not None]
        if rated_runs:
            lines.append(f"Rated runs: {len(rated_runs)} / {len(detailed_runs)}")
            avg_rating = sum(dr["rating"] for dr in rated_runs) / len(rated_runs)
            lines.append(f"Average rating: {avg_rating:.1f} / 5")
            lines.append("")
            lines.append("| Run | Rating | Notes |")
            lines.append("|-----|--------|-------|")
            for dr in rated_runs:
                notes = dr.get("rating_notes") or ""
                lines.append(
                    f"| #{dr['run_number']} | {dr['rating']}/5 | {notes} |"
                )
            lines.append("")
        else:
            lines.append("No human ratings recorded.")
            lines.append("")

        # --- Baseline Configuration ---
        lines.append("## Baseline Configuration")
        lines.append("")
        lines.append("### SLAM Parameters")
        lines.append("```json")
        lines.append(json.dumps(base_slam, indent=2))
        lines.append("```")
        lines.append("")
        lines.append("### EKF Parameters")
        lines.append("```json")
        lines.append(json.dumps(base_ekf, indent=2))
        lines.append("```")
        lines.append("")

        # --- Recommended Configuration ---
        lines.append("## Recommended Configuration")
        lines.append("")
        if best_run:
            try:
                best_slam = json.loads(best_run.get("slam_params", "{}"))
            except (json.JSONDecodeError, TypeError):
                best_slam = {}
            try:
                best_ekf = json.loads(best_run.get("ekf_params", "{}"))
            except (json.JSONDecodeError, TypeError):
                best_ekf = {}
            recommended_slam = {**base_slam, **best_slam}
            recommended_ekf = {**base_ekf, **best_ekf}
            lines.append("### SLAM Parameters")
            lines.append("```json")
            lines.append(json.dumps(recommended_slam, indent=2))
            lines.append("```")
            lines.append("")
            lines.append("### EKF Parameters")
            lines.append("```json")
            lines.append(json.dumps(recommended_ekf, indent=2))
            lines.append("```")
        else:
            lines.append("No completed analysis available for recommendation.")
        lines.append("")

        # --- Raw Data Reference ---
        lines.append("## Raw Data Reference")
        lines.append(f"Database: {self._db_path}")
        lines.append(f"Session ID: {session_id}")
        lines.append("")

        report_content = "\n".join(lines)

        # Write to file
        if output_dir is None:
            output_dir = os.path.dirname(self._db_path)
        os.makedirs(output_dir, exist_ok=True)

        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        safe_name = session["name"].replace(" ", "_").replace("/", "_")
        filename = f"autotune_report_{safe_name}_{timestamp}.md"
        filepath = os.path.join(output_dir, filename)

        with open(filepath, "w") as f:
            f.write(report_content)

        self._log_info(f"Report exported to {filepath}")
        return filepath


def export_report_cli():
    """Standalone CLI entry point for exporting a session report."""
    import argparse

    parser = argparse.ArgumentParser(description="Export autotune session report")
    parser.add_argument(
        "--db-path",
        default="/opt/mapping_autotune/autotune.db",
        help="Path to the autotune SQLite database",
    )
    parser.add_argument(
        "--session-id",
        type=int,
        required=True,
        help="Session ID to export",
    )
    parser.add_argument(
        "--output-dir",
        default=None,
        help="Directory to write the report to (defaults to db directory)",
    )
    args = parser.parse_args()

    db = DbManager(args.db_path)
    path = db.export_session_report(args.session_id, args.output_dir)
    print(f"Report saved to: {path}")
