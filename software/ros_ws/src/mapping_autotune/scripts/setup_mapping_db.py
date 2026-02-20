#!/usr/bin/env python3
"""Standalone database setup for mapping autotune — Python alternative.

No ROS dependency. Works on any system with Python 3 and sqlite3.
"""

import argparse
import json
import os
import sqlite3
import subprocess
import sys

SCHEMA = """
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


def create_schema(db_path):
    conn = sqlite3.connect(db_path)
    conn.executescript(SCHEMA)
    conn.close()


def setup_local(db_dir):
    db_path = os.path.join(db_dir, "autotune.db")
    print("=== Mapping Autotune Database Setup ===\n")

    os.makedirs(db_dir, exist_ok=True)

    if os.path.exists(db_path):
        print(f"Database already exists at: {db_path}")
        print("Running schema migration (safe — IF NOT EXISTS)...")
    else:
        print(f"Creating database: {db_path}")

    create_schema(db_path)

    print(f"\nDatabase ready at: {db_path}")
    print("\nTo use with autotune_node:")
    print(f"  ros2 run mapping_autotune autotune_node --ros-args -p db_path:={db_path}")
    print("\nTo use with review_tui:")
    print(f"  ros2 run mapping_autotune review_tui --ros-args -p db_path:={db_path}")


def setup_server(db_dir):
    db_path = os.path.join(db_dir, "autotune.db")
    print("=== Mapping Autotune Server Setup ===\n")

    os.makedirs(db_dir, exist_ok=True)
    create_schema(db_path)

    print(f"Server database ready at: {db_path}")
    print("\nTo review results on this server (no ROS needed):")
    print(f"  python3 review_tui_standalone.py --db-path {db_path}")


def setup_remote(db_dir, ssh_host, ssh_user, ssh_key, remote_db_dir, sync_interval):
    db_path = os.path.join(db_dir, "autotune.db")
    print("=== Mapping Autotune Remote Sync Setup ===\n")

    os.makedirs(db_dir, exist_ok=True)
    create_schema(db_path)

    # Test SSH
    print(f"Testing SSH connection to {ssh_user}@{ssh_host}...")
    result = subprocess.run(
        [
            "ssh",
            "-i",
            ssh_key,
            "-o",
            "ConnectTimeout=5",
            "-o",
            "BatchMode=yes",
            f"{ssh_user}@{ssh_host}",
            "echo ok",
        ],
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        print("  SSH connection: FAILED")
        print(f"  Try: ssh-copy-id -i {ssh_key} {ssh_user}@{ssh_host}")
        sys.exit(1)
    print("  SSH connection: OK")

    # Write sync config
    sync_config = {
        "ssh_host": ssh_host,
        "ssh_user": ssh_user,
        "ssh_key": ssh_key,
        "remote_db_dir": remote_db_dir,
        "sync_interval": sync_interval,
    }
    config_path = os.path.join(db_dir, "sync_config.json")
    with open(config_path, "w") as f:
        json.dump(sync_config, f, indent=4)

    print("\nSync configured:")
    print(f"  Local DB:    {db_path}")
    print(f"  Remote:      {ssh_user}@{ssh_host}:{remote_db_dir}/autotune.db")
    print(f"  Sync config: {config_path}")


def main():
    parser = argparse.ArgumentParser(description="Setup mapping autotune database")
    parser.add_argument(
        "--db-dir",
        default="/opt/mapping_autotune",
        help="Database directory (default: /opt/mapping_autotune)",
    )
    parser.add_argument(
        "--server", action="store_true", help="Server mode: create DB for remote review"
    )
    parser.add_argument(
        "--remote-sync",
        action="store_true",
        help="Configure sync from robot to remote server",
    )
    parser.add_argument("--ssh-host", default="", help="SSH host for remote sync")
    parser.add_argument("--ssh-user", default="", help="SSH user for remote sync")
    parser.add_argument(
        "--ssh-key", default="~/.ssh/id_ed25519", help="SSH private key path"
    )
    parser.add_argument(
        "--remote-db-dir", default="", help="Database directory on remote server"
    )
    parser.add_argument(
        "--sync-interval",
        default="every_run",
        choices=["every_run", "end_of_session", "manual"],
        help="Sync frequency",
    )
    args = parser.parse_args()

    if args.remote_sync:
        if not args.ssh_host or not args.ssh_user:
            parser.error("--ssh-host and --ssh-user required for --remote-sync")
        remote_dir = args.remote_db_dir or args.db_dir
        setup_remote(
            args.db_dir,
            args.ssh_host,
            args.ssh_user,
            os.path.expanduser(args.ssh_key),
            remote_dir,
            args.sync_interval,
        )
    elif args.server:
        setup_server(args.db_dir)
    else:
        setup_local(args.db_dir)


if __name__ == "__main__":
    main()
