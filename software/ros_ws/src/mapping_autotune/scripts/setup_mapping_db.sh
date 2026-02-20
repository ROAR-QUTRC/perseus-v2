#!/usr/bin/env bash
# setup_mapping_db.sh — Standalone database setup for mapping autotune
# No ROS dependency. Works on any Ubuntu 24.04 machine with sqlite3.
set -euo pipefail

DB_DIR="/opt/mapping_autotune"
DB_NAME="autotune.db"
MODE="local"
SSH_HOST=""
SSH_USER=""
SSH_KEY=""
REMOTE_DB_DIR=""
SYNC_INTERVAL="every_run"

usage() {
  cat <<EOF
Usage: $0 [OPTIONS]

Sets up the SQLite database for the mapping autotune system.

OPTIONS:
  --db-dir DIR          Database directory (default: /opt/mapping_autotune)
  --server              Server mode: create DB for remote review
  --remote-sync         Configure sync from robot to remote server
  --ssh-host HOST       SSH host for remote sync
  --ssh-user USER       SSH user for remote sync
  --ssh-key PATH        SSH private key path
  --remote-db-dir DIR   Database directory on remote server
  --sync-interval INT   Sync frequency: every_run|end_of_session|manual (default: every_run)
  -h, --help            Show this help

EXAMPLES:
  # On the robot (local setup):
  $0

  # On a review server:
  $0 --server --db-dir /srv/mapping_autotune

  # On the robot (configure remote sync):
  $0 --remote-sync --ssh-host 192.168.1.100 --ssh-user mapper --ssh-key ~/.ssh/id_ed25519
EOF
  exit 0
}

while [[ $# -gt 0 ]]; do
  case "$1" in
  --db-dir)
    DB_DIR="$2"
    shift 2
    ;;
  --server)
    MODE="server"
    shift
    ;;
  --remote-sync)
    MODE="remote"
    shift
    ;;
  --ssh-host)
    SSH_HOST="$2"
    shift 2
    ;;
  --ssh-user)
    SSH_USER="$2"
    shift 2
    ;;
  --ssh-key)
    SSH_KEY="$2"
    shift 2
    ;;
  --remote-db-dir)
    REMOTE_DB_DIR="$2"
    shift 2
    ;;
  --sync-interval)
    SYNC_INTERVAL="$2"
    shift 2
    ;;
  -h | --help) usage ;;
  *)
    echo "Unknown option: $1"
    usage
    ;;
  esac
done

DB_PATH="${DB_DIR}/${DB_NAME}"

create_schema() {
  local db="$1"
  sqlite3 "$db" <<'SQL'
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
SQL
}

# --- Local setup ---
setup_local() {
  echo "=== Mapping Autotune Database Setup ==="
  echo ""

  # Create directory
  if [[ ! -d $DB_DIR ]]; then
    echo "Creating directory: $DB_DIR"
    sudo mkdir -p "$DB_DIR"
    sudo chown "$(whoami):$(whoami)" "$DB_DIR"
  fi

  # Create database
  if [[ -f $DB_PATH ]]; then
    echo "Database already exists at: $DB_PATH"
    echo "Running schema migration (safe — IF NOT EXISTS)..."
  else
    echo "Creating database: $DB_PATH"
  fi

  create_schema "$DB_PATH"
  chmod 664 "$DB_PATH"

  echo ""
  echo "Database ready at: $DB_PATH"
  echo ""
  echo "To use with autotune_node:"
  echo "  ros2 run mapping_autotune autotune_node --ros-args -p db_path:=$DB_PATH"
  echo ""
  echo "To use with review_tui:"
  echo "  ros2 run mapping_autotune review_tui --ros-args -p db_path:=$DB_PATH"
}

# --- Server setup ---
setup_server() {
  echo "=== Mapping Autotune Server Setup ==="
  echo ""

  if [[ ! -d $DB_DIR ]]; then
    echo "Creating directory: $DB_DIR"
    sudo mkdir -p "$DB_DIR"
    sudo chown "$(whoami):$(whoami)" "$DB_DIR"
  fi

  create_schema "$DB_PATH"
  chmod 664 "$DB_PATH"

  echo ""
  echo "Server database ready at: $DB_PATH"
  echo ""
  echo "Robot sync command:"
  echo '  ./setup_mapping_db.sh --remote-sync \'
  echo "    --ssh-host $(hostname -I | awk '{print $1}') \\"
  echo "    --ssh-user $(whoami) \\"
  echo '    --ssh-key ~/.ssh/id_ed25519 \'
  echo "    --remote-db-dir $DB_DIR"
  echo ""
  echo "To review results on this server (no ROS needed):"
  echo "  python3 review_tui_standalone.py --db-path $DB_PATH"
}

# --- Remote sync setup ---
setup_remote() {
  if [[ -z $SSH_HOST || -z $SSH_USER ]]; then
    echo "ERROR: --ssh-host and --ssh-user are required for --remote-sync"
    exit 1
  fi

  [[ -z $SSH_KEY ]] && SSH_KEY="$HOME/.ssh/id_ed25519"
  [[ -z $REMOTE_DB_DIR ]] && REMOTE_DB_DIR="$DB_DIR"

  echo "=== Mapping Autotune Remote Sync Setup ==="
  echo ""

  # Create local directory and database
  if [[ ! -d $DB_DIR ]]; then
    echo "Creating local directory: $DB_DIR"
    sudo mkdir -p "$DB_DIR"
    sudo chown "$(whoami):$(whoami)" "$DB_DIR"
  fi

  create_schema "$DB_PATH"

  # Test SSH connectivity
  echo "Testing SSH connection to $SSH_USER@$SSH_HOST..."
  if ssh -i "$SSH_KEY" -o ConnectTimeout=5 -o BatchMode=yes "$SSH_USER@$SSH_HOST" "echo ok" >/dev/null 2>&1; then
    echo "  SSH connection: OK"
  else
    echo "  SSH connection: FAILED"
    echo "  Ensure SSH key is authorized on the server."
    echo "  Try: ssh-copy-id -i $SSH_KEY $SSH_USER@$SSH_HOST"
    exit 1
  fi

  # Test rsync
  echo "Testing rsync to $SSH_USER@$SSH_HOST:$REMOTE_DB_DIR/..."
  if rsync -az --dry-run "$DB_PATH" "$SSH_USER@$SSH_HOST:$REMOTE_DB_DIR/" -e "ssh -i $SSH_KEY" >/dev/null 2>&1; then
    echo "  Rsync test: OK"
  else
    echo "  Rsync test: FAILED"
    echo "  Ensure the remote directory exists and is writable."
    exit 1
  fi

  # Write sync config
  SYNC_CONFIG="$DB_DIR/sync_config.json"
  cat >"$SYNC_CONFIG" <<EOF
{
    "ssh_host": "$SSH_HOST",
    "ssh_user": "$SSH_USER",
    "ssh_key": "$SSH_KEY",
    "remote_db_dir": "$REMOTE_DB_DIR",
    "sync_interval": "$SYNC_INTERVAL"
}
EOF

  echo ""
  echo "Sync configured:"
  echo "  Local DB:    $DB_PATH"
  echo "  Remote:      $SSH_USER@$SSH_HOST:$REMOTE_DB_DIR/$DB_NAME"
  echo "  Sync config: $SYNC_CONFIG"
  echo "  Interval:    $SYNC_INTERVAL"
  echo ""
  echo "The autotune node will auto-detect this config on startup."
}

# --- Main ---
case "$MODE" in
local) setup_local ;;
server) setup_server ;;
remote) setup_remote ;;
esac
