"""Tests for db_manager module using a temporary in-memory database."""
import json
import os
import tempfile
import zlib

import pytest

from mapping_autotune.db_manager import DbManager


@pytest.fixture
def db():
    """Create a DbManager with a temp file database."""
    with tempfile.NamedTemporaryFile(suffix=".db", delete=False) as f:
        db_path = f.name
    manager = DbManager(db_path)
    yield manager
    os.unlink(db_path)


@pytest.fixture
def session_id(db):
    """Create a test session and return its ID."""
    return db.create_session(
        name="test_session",
        description="Test session for unit tests",
        base_slam_config=json.dumps({"minimum_travel_heading": 0.2}),
        base_ekf_config=json.dumps({"frequency": 15.0}),
    )


class TestDbManager:
    def test_check_connection(self, db):
        assert db.check_connection() is True

    def test_create_session(self, db):
        sid = db.create_session(
            name="my_session",
            description="A test",
            base_slam_config="{}",
            base_ekf_config="{}",
        )
        assert isinstance(sid, int)
        assert sid > 0

    def test_get_session(self, db, session_id):
        session = db.get_session(session_id)
        assert session is not None
        assert session["name"] == "test_session"
        assert session["status"] == "active"

    def test_update_session_status(self, db, session_id):
        db.update_session_status(session_id, "completed")
        session = db.get_session(session_id)
        assert session["status"] == "completed"

    def test_create_run(self, db, session_id):
        run_id = db.create_run(
            session_id=session_id,
            run_number=1,
            slam_params=json.dumps({"minimum_travel_heading": 0.1}),
            ekf_params=json.dumps({}),
        )
        assert isinstance(run_id, int)
        assert run_id > 0

    def test_update_run_status(self, db, session_id):
        run_id = db.create_run(session_id, 1, "{}", "{}")
        db.update_run_status(run_id, "running")
        run = db.get_run(run_id)
        assert run["status"] == "running"

    def test_store_map_data(self, db, session_id):
        run_id = db.create_run(session_id, 1, "{}", "{}")
        map_data = zlib.compress(bytes([0] * 100))
        db.store_map_data(run_id, map_data, 10, 10, 0.05)
        run = db.get_run(run_id)
        assert run["map_width"] == 10
        assert run["map_height"] == 10
        assert run["map_resolution"] == 0.05

    def test_store_analysis(self, db, session_id):
        run_id = db.create_run(session_id, 1, "{}", "{}")
        metrics = {
            "wall_straightness": 0.91,
            "wall_thickness": 0.85,
            "ghost_wall_score": 0.82,
            "symmetry_score": 0.85,
            "free_space_consistency": 0.92,
            "occupied_density_score": 0.88,
            "composite_score": 0.87,
            "diagnostics": json.dumps({"lines_detected": 12}),
        }
        db.store_analysis(run_id, metrics)
        run = db.get_run(run_id)
        assert run["composite_score"] == pytest.approx(0.87)

    def test_store_rating(self, db, session_id):
        run_id = db.create_run(session_id, 1, "{}", "{}")
        db.store_rating(run_id, 4, notes="Good map quality")
        run = db.get_run(run_id)
        assert run["rating"] == 4
        assert run["rating_notes"] == "Good map quality"

    def test_store_rating_replace(self, db, session_id):
        run_id = db.create_run(session_id, 1, "{}", "{}")
        db.store_rating(run_id, 3)
        db.store_rating(run_id, 5, notes="Updated")
        run = db.get_run(run_id)
        assert run["rating"] == 5

    def test_get_runs(self, db, session_id):
        for i in range(3):
            db.create_run(session_id, i + 1, "{}", "{}")
        runs = db.get_runs(session_id)
        assert len(runs) == 3

    def test_get_best_run(self, db, session_id):
        for i, score in enumerate([0.5, 0.9, 0.7]):
            run_id = db.create_run(session_id, i + 1, "{}", "{}")
            db.store_analysis(run_id, {
                "wall_straightness": score,
                "wall_thickness": score,
                "ghost_wall_score": score,
                "symmetry_score": score,
                "free_space_consistency": score,
                "occupied_density_score": score,
                "composite_score": score,
                "diagnostics": "{}",
            })
        best = db.get_best_run(session_id)
        assert best is not None
        assert best["composite_score"] == pytest.approx(0.9)

    def test_get_all_sessions(self, db):
        db.create_session("s1", "", "{}", "{}")
        db.create_session("s2", "", "{}", "{}")
        sessions = db.get_all_sessions()
        assert len(sessions) >= 2

    def test_store_sensor_logs(self, db, session_id):
        run_id = db.create_run(session_id, 1, "{}", "{}")
        odom = json.dumps([{"t": 0.0, "x": 0.0, "y": 0.0}])
        imu = json.dumps([{"t": 0.0, "gx": 0.0}])
        db.store_sensor_logs(run_id, odom, imu)
        run = db.get_run(run_id)
        assert json.loads(run["odom_log"]) == [{"t": 0.0, "x": 0.0, "y": 0.0}]

    def test_export_session_report(self, db, session_id):
        run_id = db.create_run(session_id, 1,
                               json.dumps({"minimum_travel_heading": 0.1}), "{}")
        db.store_analysis(run_id, {
            "wall_straightness": 0.9,
            "wall_thickness": 0.8,
            "ghost_wall_score": 0.7,
            "symmetry_score": 0.6,
            "free_space_consistency": 0.5,
            "occupied_density_score": 0.4,
            "composite_score": 0.65,
            "diagnostics": "{}",
        })
        db.update_run_status(run_id, "completed")

        with tempfile.TemporaryDirectory() as tmpdir:
            path = db.export_session_report(session_id, output_dir=tmpdir)
            assert os.path.exists(path)
            with open(path) as f:
                content = f.read()
            assert "test_session" in content
            assert "0.65" in content
            assert "minimum_travel_heading" in content
