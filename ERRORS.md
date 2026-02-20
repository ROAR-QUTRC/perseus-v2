# Error Log

### Ghost wall detection used meter-scale thresholds in pixel-space — 2026-02-20

- **Severity:** Medium
- **Category:** Logic
- **File(s):** `software/ros_ws/src/mapping_autotune/mapping_autotune/map_analyzer.py`
- **Pattern:** Using `resolution` (meters/cell) as a multiplier for thresholds compared against pixel-coordinate distances from OpenCV functions like `HoughLinesP`.
- **Root cause:** `_ghost_wall_score` computed `min_dist = 2.0 * resolution` and `max_dist = 8.0 * resolution`. With resolution=0.05m, this gave thresholds of 0.1 and 0.4 pixels — far too small to detect ghost walls 3 cells apart. HoughLinesP returns coordinates in pixel (cell) units, not meters.
- **Fix applied:** Changed thresholds to direct cell counts: `min_dist = 2.0` and `max_dist = 10.0`.
- **Prevention rule:** When working with OpenCV geometry functions (HoughLinesP, distanceTransform, etc.), always verify whether output coordinates are in pixel or world units before applying distance thresholds.

### Run allocator could not trim single-run phases — 2026-02-20

- **Severity:** Low
- **Category:** Logic
- **File(s):** `software/ros_ws/src/mapping_autotune/mapping_autotune/param_manager.py`
- **Pattern:** Budget-trimming loop that floors allocations at 1 when the total budget is less than the number of active phases.
- **Root cause:** `allocate_runs(5)` gave every non-zero-weight phase `max(1, ...)` = 1 run each (6 phases = 6 runs). The trimming loop used `can_trim = max(0, allocation - 1)` which was always 0, making it impossible to reduce the total below 6.
- **Fix applied:** Changed trimming to allow reduction to 0: `trim = min(excess, allocation[phase_num])`.
- **Prevention rule:** When implementing budget allocation with a trim-down pass, ensure the trim floor matches the actual minimum (0 if phases can be skipped, not 1).
