"""Parameter generation and management for mapping autotune phases."""

import copy
import itertools
import tempfile

import yaml


# Phase definitions: (phase_number, name, default_run_count, priority_weight)
PHASE_DEFS = [
    (0, "imu_calibration", 0, 0),
    (1, "imu_integration", 3, 3),
    (2, "slam_rotation", 10, 3),
    (3, "scan_matching", 6, 3),
    (4, "timing", 4, 2),
    (5, "speed_limits", 3, 1),
    (6, "ekf_noise", 3, 1),
]

# Process noise covariance matrix dimensions
_PNC_SIZE = 15
_PNC_YAW_IDX = 5
_PNC_VYAW_IDX = 11


def _pnc_diag_offset(idx):
    """Flat index into the 15x15 process_noise_covariance for diagonal element."""
    return idx * _PNC_SIZE + idx


class ParamManager:
    def __init__(self, baseline_slam: dict, baseline_ekf: dict, defaults: dict):
        self._baseline_slam = copy.deepcopy(baseline_slam)
        self._baseline_ekf = copy.deepcopy(baseline_ekf)
        self._defaults = defaults

        # Extract the ros__parameters sections for easy access
        self._slam_params = self._baseline_slam.get(
            "slam_toolbox", {}
        ).get("ros__parameters", {})
        self._ekf_params = self._baseline_ekf.get(
            "ekf_filter_node", {}
        ).get("ros__parameters", {})

    def get_phase_runs(self, phase_number: int, best_params_so_far: dict = None) -> list:
        """Return a list of parameter dicts for the given phase.

        Each dict contains all keys that differ from the baseline for this run,
        plus a 'phase' and 'run_label' key for identification.
        """
        if best_params_so_far is None:
            best_params_so_far = {}

        generators = {
            0: self._phase0_runs,
            1: self._phase1_runs,
            2: self._phase2_runs,
            3: self._phase3_runs,
            4: self._phase4_runs,
            5: self._phase5_runs,
            6: self._phase6_runs,
        }

        generator = generators.get(phase_number)
        if generator is None:
            return []

        return generator(best_params_so_far)

    def apply_slam_params(self, params_dict: dict) -> str:
        """Write a temp YAML file with SLAM params merged on top of baseline.

        Returns the path to the temp file.
        """
        merged = copy.deepcopy(self._slam_params)
        slam_overrides = params_dict.get("slam", {})
        merged.update(slam_overrides)

        config = {"slam_toolbox": {"ros__parameters": merged}}
        return self._write_temp_yaml(config, prefix="slam_")

    def apply_ekf_params(self, params_dict: dict) -> str:
        """Write a temp YAML file with EKF params merged on top of baseline.

        Returns the path to the temp file.
        """
        merged = copy.deepcopy(self._ekf_params)
        ekf_overrides = params_dict.get("ekf", {})

        # Handle process_noise_covariance specially — allow partial updates
        if "process_noise_covariance_overrides" in ekf_overrides:
            pnc = list(merged.get("process_noise_covariance", [0.0] * 225))
            for offset, value in ekf_overrides.pop("process_noise_covariance_overrides").items():
                pnc[int(offset)] = value
            merged["process_noise_covariance"] = pnc

        merged.update(ekf_overrides)

        config = {"ekf_filter_node": {"ros__parameters": merged}}
        return self._write_temp_yaml(config, prefix="ekf_")

    @staticmethod
    def get_param_diff(baseline: dict, current: dict) -> dict:
        """Return only keys from current that differ from baseline."""
        diff = {}
        for key, value in current.items():
            if key not in baseline or baseline[key] != value:
                diff[key] = value
        return diff

    @staticmethod
    def allocate_runs(max_runs: int) -> dict:
        """Allocate run counts per phase given a total budget.

        Priority goes to phases 1-3, then 4-6.
        """
        # Phase 0 always gets 0 runs (calibration only)
        total_weight = sum(w for _, _, _, w in PHASE_DEFS if w > 0)
        if total_weight == 0:
            return {p: 0 for p, _, _, _ in PHASE_DEFS}

        allocation = {}

        # First pass: proportional allocation
        for phase_num, _, default_count, weight in PHASE_DEFS:
            if weight == 0:
                allocation[phase_num] = 0
                continue
            allocated = max(1, round(max_runs * weight / total_weight))
            # Don't exceed the phase's designed run count
            allocated = min(allocated, default_count)
            allocation[phase_num] = allocated

        # Adjust to fit within max_runs
        total_allocated = sum(allocation.values())
        if total_allocated > max_runs:
            # Trim from lowest priority phases first, allowing reduction to 0
            for phase_num in reversed([p for p, _, _, _ in PHASE_DEFS]):
                if total_allocated <= max_runs:
                    break
                excess = total_allocated - max_runs
                trim = min(excess, allocation[phase_num])
                allocation[phase_num] -= trim
                total_allocated -= trim

        return allocation

    # ── Phase generators ──────────────────────────────────────────────

    def _phase0_runs(self, best: dict) -> list:
        return []

    def _phase1_runs(self, best: dict) -> list:
        """IMU integration: imu_off, imu_on, imu_on+deadband."""
        runs = []

        # Run 1: IMU off (baseline — imu0_config all false)
        runs.append({
            "phase": 1,
            "run_label": "imu_off",
            "slam": {},
            "ekf": {
                "imu0_config": [
                    False, False, False, False, False, False,
                    False, False, False, False, False, False,
                    False, False, False,
                ],
            },
            "maneuver": {},
        })

        # Run 2: IMU on — enable vyaw (index 11) from IMU
        imu_on_config = [
            False, False, False, False, False, False,
            False, False, False, False, False, True,
            False, False, False,
        ]
        runs.append({
            "phase": 1,
            "run_label": "imu_on",
            "slam": {},
            "ekf": {
                "imu0_config": imu_on_config,
            },
            "maneuver": {},
        })

        # Run 3: IMU on + deadband filter
        deadband_threshold = self._defaults.get("imu_deadband_threshold", 0.01)
        runs.append({
            "phase": 1,
            "run_label": "imu_on_deadband",
            "slam": {},
            "ekf": {
                "imu0_config": imu_on_config,
            },
            "imu_filter": {
                "deadband_threshold": deadband_threshold,
                "enabled": True,
            },
            "maneuver": {},
        })

        return runs

    def _phase2_runs(self, best: dict) -> list:
        """SLAM rotation parameters: vary heading, smear, angle penalty."""
        heading_values = [0.05, 0.1, 0.2, 0.4]
        smear_values = [0.02, 0.05, 0.1]
        angle_penalty_values = [0.5, 1.0, 2.0, 3.0]

        # Use Latin-hypercube-style sampling to keep runs manageable
        # Pick representative combinations rather than full factorial (48 combos)
        combos = self._sample_combinations(
            {
                "minimum_travel_heading": heading_values,
                "correlation_search_space_smear_deviation": smear_values,
                "angle_variance_penalty": angle_penalty_values,
            },
            max_runs=10,
        )

        runs = []
        for i, combo in enumerate(combos):
            run = {
                "phase": 2,
                "run_label": f"rotation_{i:02d}",
                "slam": combo,
                "ekf": best.get("ekf", {}),
                "maneuver": {},
            }
            runs.append(run)

        return runs

    def _phase3_runs(self, best: dict) -> list:
        """Scan matching strictness parameters."""
        angle_penalty_values = [0.9, 0.95, 0.99]
        response_fine_values = [0.3, 0.4, 0.5, 0.6]
        distance_penalty_values = [0.5, 0.7, 0.9]

        combos = self._sample_combinations(
            {
                "minimum_angle_penalty": angle_penalty_values,
                "link_match_minimum_response_fine": response_fine_values,
                "minimum_distance_penalty": distance_penalty_values,
            },
            max_runs=6,
        )

        runs = []
        for i, combo in enumerate(combos):
            run = {
                "phase": 3,
                "run_label": f"scan_match_{i:02d}",
                "slam": {**best.get("slam", {}), **combo},
                "ekf": best.get("ekf", {}),
                "maneuver": {},
            }
            runs.append(run)

        return runs

    def _phase4_runs(self, best: dict) -> list:
        """Timing parameters."""
        throttle_values = [1, 2, 3, 5]
        min_time_values = [0.1, 0.3, 0.5]
        transform_period_values = [0.02, 0.05, 0.1]

        combos = self._sample_combinations(
            {
                "throttle_scans": throttle_values,
                "minimum_time_interval": min_time_values,
                "transform_publish_period": transform_period_values,
            },
            max_runs=5,
        )

        runs = []
        for i, combo in enumerate(combos):
            run = {
                "phase": 4,
                "run_label": f"timing_{i:02d}",
                "slam": {**best.get("slam", {}), **combo},
                "ekf": best.get("ekf", {}),
                "maneuver": {},
            }
            runs.append(run)

        return runs

    def _phase5_runs(self, best: dict) -> list:
        """Speed limit (maneuver) parameters."""
        rotation_speeds = [0.2, 0.3, 0.5, 1.0]
        linear_speeds = [0.1, 0.2, 0.3]

        combos = self._sample_combinations(
            {
                "rotation_speed": rotation_speeds,
                "linear_speed": linear_speeds,
            },
            max_runs=4,
        )

        runs = []
        for i, combo in enumerate(combos):
            run = {
                "phase": 5,
                "run_label": f"speed_{i:02d}",
                "slam": best.get("slam", {}),
                "ekf": best.get("ekf", {}),
                "maneuver": combo,
            }
            runs.append(run)

        return runs

    def _phase6_runs(self, best: dict) -> list:
        """EKF process noise tuning for yaw and vyaw."""
        yaw_noise_values = [0.03, 0.06, 0.12]
        vyaw_noise_values = [0.01, 0.02, 0.04]

        combos = self._sample_combinations(
            {
                "process_noise_yaw": yaw_noise_values,
                "process_noise_vyaw": vyaw_noise_values,
            },
            max_runs=4,
        )

        runs = []
        for i, combo in enumerate(combos):
            pnc_overrides = {}
            if "process_noise_yaw" in combo:
                pnc_overrides[_pnc_diag_offset(_PNC_YAW_IDX)] = combo["process_noise_yaw"]
            if "process_noise_vyaw" in combo:
                pnc_overrides[_pnc_diag_offset(_PNC_VYAW_IDX)] = combo["process_noise_vyaw"]

            run = {
                "phase": 6,
                "run_label": f"ekf_noise_{i:02d}",
                "slam": best.get("slam", {}),
                "ekf": {
                    **best.get("ekf", {}),
                    "process_noise_covariance_overrides": pnc_overrides,
                },
                "maneuver": {},
                # Store readable values for logging
                "ekf_noise_readable": combo,
            }
            runs.append(run)

        return runs

    # ── Helpers ────────────────────────────────────────────────────────

    @staticmethod
    def _sample_combinations(param_grid: dict, max_runs: int) -> list:
        """Sample representative combinations from a parameter grid.

        If the full factorial is within max_runs, return all combinations.
        Otherwise, use a diagonal/spread sampling strategy to cover the space.
        """
        keys = list(param_grid.keys())
        value_lists = [param_grid[k] for k in keys]
        full_product = list(itertools.product(*value_lists))

        if len(full_product) <= max_runs:
            return [dict(zip(keys, combo)) for combo in full_product]

        # Spread sampling: pick evenly spaced indices from the full product
        # First, sort by a pseudo-distance metric to ensure diversity
        step = len(full_product) / max_runs
        indices = [int(round(i * step)) for i in range(max_runs)]
        # Clamp to valid range
        indices = [min(i, len(full_product) - 1) for i in indices]
        # Remove duplicates while preserving order
        seen = set()
        unique_indices = []
        for idx in indices:
            if idx not in seen:
                seen.add(idx)
                unique_indices.append(idx)

        # If we lost some due to dedup, fill from remaining
        all_indices = set(range(len(full_product)))
        remaining = sorted(all_indices - seen)
        while len(unique_indices) < max_runs and remaining:
            unique_indices.append(remaining.pop(0))

        return [dict(zip(keys, full_product[i])) for i in unique_indices[:max_runs]]

    @staticmethod
    def _write_temp_yaml(config: dict, prefix: str = "autotune_") -> str:
        """Write config dict to a temp YAML file and return its path."""
        f = tempfile.NamedTemporaryFile(
            suffix=".yaml",
            prefix=prefix,
            delete=False,
            mode="w",
        )
        yaml.dump(config, f, default_flow_style=False, sort_keys=False)
        f.close()
        return f.name
