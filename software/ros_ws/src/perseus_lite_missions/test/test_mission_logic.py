"""Unit tests for the pure mission logic (no rclpy required)."""

from perseus_lite_missions.checkers.map_quality_node import evaluate_slam_params
from perseus_lite_missions.orchestrator_node import (
    find_segment_index,
    merge_unlocked_badges,
)


def _slam_doc(resolution, max_laser_range):
    return {
        "slam_toolbox": {
            "ros__parameters": {
                "resolution": resolution,
                "max_laser_range": max_laser_range,
            }
        }
    }


class TestEvaluateSlamParams:
    def test_reference_values_pass(self):
        assert evaluate_slam_params(_slam_doc(0.05, 12.0), 10.0, 8.0)

    def test_broken_starting_values_fail(self):
        assert not evaluate_slam_params(_slam_doc(0.30, 3.0), 10.0, 8.0)

    def test_threshold_boundary_passes(self):
        assert evaluate_slam_params(_slam_doc(0.1, 8.0), 10.0, 8.0)

    def test_coarse_resolution_fails(self):
        assert not evaluate_slam_params(_slam_doc(0.2, 12.0), 10.0, 8.0)

    def test_short_range_fails(self):
        assert not evaluate_slam_params(_slam_doc(0.05, 3.0), 10.0, 8.0)

    def test_zero_resolution_fails(self):
        assert not evaluate_slam_params(_slam_doc(0.0, 12.0), 10.0, 8.0)

    def test_missing_keys_fail(self):
        assert not evaluate_slam_params({}, 10.0, 8.0)
        assert not evaluate_slam_params(None, 10.0, 8.0)
        assert not evaluate_slam_params(
            {"slam_toolbox": {"ros__parameters": {"resolution": 0.05}}}, 10.0, 8.0
        )


class TestFindSegmentIndex:
    SEGMENTS = [{"id": 1, "title": "a"}, {"id": 5, "title": "b"}, {"id": 8}]

    def test_finds_segment(self):
        assert find_segment_index(self.SEGMENTS, 5) == 1
        assert find_segment_index(self.SEGMENTS, 8) == 2

    def test_missing_segment_returns_none(self):
        assert find_segment_index(self.SEGMENTS, 99) is None
        assert find_segment_index([], 1) is None


class TestMergeUnlockedBadges:
    def test_merge_into_empty(self):
        out = merge_unlocked_badges({}, ["a", "b"])
        assert out["unlocked"] == ["a", "b"]

    def test_preserves_existing_and_dedupes(self):
        out = merge_unlocked_badges({"unlocked": ["a"]}, ["a", "b"])
        assert out["unlocked"] == ["a", "b"]

    def test_non_dict_input_is_reset(self):
        out = merge_unlocked_badges(["corrupt"], ["a"])
        assert out["unlocked"] == ["a"]

    def test_preserves_unrelated_keys(self):
        out = merge_unlocked_badges({"other": 1, "unlocked": []}, ["a"])
        assert out["other"] == 1
