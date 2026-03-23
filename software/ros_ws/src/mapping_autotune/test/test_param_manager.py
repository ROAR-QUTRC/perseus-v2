"""Tests for param_manager module."""

import os
import yaml

import pytest

from mapping_autotune.param_manager import ParamManager


BASELINE_SLAM = {
    "slam_toolbox": {
        "ros__parameters": {
            "minimum_travel_heading": 0.2,
            "correlation_search_space_smear_deviation": 0.05,
            "angle_variance_penalty": 1.0,
            "minimum_angle_penalty": 0.95,
            "link_match_minimum_response_fine": 0.4,
            "minimum_distance_penalty": 0.7,
            "throttle_scans": 3,
            "minimum_time_interval": 0.5,
            "transform_publish_period": 0.05,
            "use_imu": True,
        }
    }
}

BASELINE_EKF = {
    "ekf_filter_node": {
        "ros__parameters": {
            "frequency": 15.0,
            "two_d_mode": True,
            "odom0_config": [True] * 15,
            "imu0_config": [False] * 15,
            "process_noise_covariance": [0.0] * 225,
        }
    }
}


@pytest.fixture
def defaults():
    return {}


@pytest.fixture
def pm(defaults):
    return ParamManager(BASELINE_SLAM, BASELINE_EKF, defaults)


class TestParamManager:
    def test_phase0_returns_empty(self, pm):
        runs = pm.get_phase_runs(0, {})
        assert len(runs) == 0

    def test_phase1_returns_3_runs(self, pm):
        runs = pm.get_phase_runs(1, {})
        assert len(runs) == 3
        # Should have imu_off, imu_on, imu_on_deadband
        labels = [r["run_label"] for r in runs]
        assert any("off" in label.lower() for label in labels)
        assert any("on" in label.lower() for label in labels)

    def test_phase2_returns_runs(self, pm):
        runs = pm.get_phase_runs(2, {})
        assert len(runs) > 0
        # Each run should have SLAM param overrides
        for r in runs:
            assert "slam" in r

    def test_phase5_has_maneuver_params(self, pm):
        runs = pm.get_phase_runs(5, {})
        assert len(runs) > 0
        for r in runs:
            assert "maneuver" in r

    def test_phase6_has_ekf_params(self, pm):
        runs = pm.get_phase_runs(6, {})
        assert len(runs) > 0

    def test_apply_slam_params_writes_yaml(self, pm):
        runs = pm.get_phase_runs(2, {})
        if not runs:
            pytest.skip("No phase 2 runs generated")
        path = pm.apply_slam_params(runs[0])
        assert os.path.exists(path)
        with open(path) as f:
            data = yaml.safe_load(f)
        assert "slam_toolbox" in data
        assert "ros__parameters" in data["slam_toolbox"]
        os.unlink(path)

    def test_apply_ekf_params_writes_yaml(self, pm):
        runs = pm.get_phase_runs(1, {})
        if not runs:
            pytest.skip("No phase 1 runs generated")
        path = pm.apply_ekf_params(runs[0])
        assert os.path.exists(path)
        with open(path) as f:
            data = yaml.safe_load(f)
        assert "ekf_filter_node" in data
        os.unlink(path)

    def test_get_param_diff(self):
        baseline = {"a": 1, "b": 2, "c": 3}
        current = {"a": 1, "b": 5, "c": 3}
        diff = ParamManager.get_param_diff(baseline, current)
        assert diff == {"b": 5}

    def test_get_param_diff_new_keys(self):
        baseline = {"a": 1}
        current = {"a": 1, "b": 2}
        diff = ParamManager.get_param_diff(baseline, current)
        assert diff == {"b": 2}

    def test_allocate_runs_respects_max(self):
        allocation = ParamManager.allocate_runs(5)
        total = sum(allocation.values())
        assert total <= 5

    def test_allocate_runs_full_budget(self):
        allocation = ParamManager.allocate_runs(50)
        total = sum(allocation.values())
        assert total <= 50
