import math
import zlib

import cv2
import numpy as np


class MapAnalyzer:
    """Analyzes OccupancyGrid maps for quality metrics."""

    _DEFAULT_WEIGHTS = {
        "wall_straightness": 0.25,
        "wall_thickness": 0.20,
        "ghost_wall_score": 0.20,
        "symmetry_score": 0.15,
        "free_space_consistency": 0.10,
        "occupied_density_score": 0.10,
    }

    def __init__(self, weights=None):
        self._weights = dict(self._DEFAULT_WEIGHTS)
        if weights is not None:
            self._weights.update(weights)

    def analyze(self, map_data, width, height, resolution):
        """Analyze an occupancy grid and return quality metrics.

        Args:
            map_data: Flat list/array of int8 occupancy values
                      (-1=unknown, 0=free, 100=occupied).
            width: Grid width in cells.
            height: Grid height in cells.
            resolution: Cell size in meters.

        Returns:
            Dict with individual metric scores (0.0-1.0, higher=better),
            a weighted composite_score, and diagnostics.
        """
        occupancy_grid = np.array(map_data, dtype=np.int8).reshape((height, width))
        binary_map = np.where(occupancy_grid == 100, 255, 0).astype(np.uint8)

        scores = {
            "wall_straightness": self._wall_straightness(binary_map, resolution),
            "wall_thickness": self._wall_thickness(binary_map, resolution),
            "ghost_wall_score": self._ghost_wall_score(binary_map, resolution),
            "symmetry_score": self._symmetry_score(occupancy_grid),
            "free_space_consistency": self._free_space_consistency(
                binary_map, occupancy_grid
            ),
            "occupied_density_score": self._occupied_density_score(
                binary_map, occupancy_grid
            ),
        }

        composite = sum(
            self._weights.get(key, 0.0) * value for key, value in scores.items()
        )

        total_occupied = int(np.count_nonzero(binary_map))
        total_known = int(np.count_nonzero(occupancy_grid != -1))
        total_unknown = int(np.count_nonzero(occupancy_grid == -1))

        diagnostics = {
            "total_occupied": total_occupied,
            "total_known": total_known,
            "total_unknown": total_unknown,
            "width": width,
            "height": height,
            "resolution": resolution,
        }

        return {**scores, "composite_score": composite, "diagnostics": diagnostics}

    # ------------------------------------------------------------------
    # Metric methods
    # ------------------------------------------------------------------

    def _wall_straightness(self, binary_map, resolution):
        """Score how straight the detected walls are.

        Uses Hough line detection and measures perpendicular deviation of
        nearby occupied cells from each detected segment.
        """
        lines = cv2.HoughLinesP(
            binary_map,
            rho=1,
            theta=np.pi / 180,
            threshold=20,
            minLineLength=10,
            maxLineGap=5,
        )
        if lines is None or len(lines) == 0:
            return 0.5

        occupied_points = np.argwhere(binary_map == 255)  # (row, col)
        if len(occupied_points) == 0:
            return 0.5

        max_acceptable = 3.0 * resolution
        deviations = []

        for line in lines:
            x1, y1, x2, y2 = line[0]
            dx = float(x2 - x1)
            dy = float(y2 - y1)
            length = math.hypot(dx, dy)
            if length < 1e-6:
                continue

            # Unit normal to the line segment
            nx = -dy / length
            ny = dx / length

            # Midpoint of segment
            mx = (x1 + x2) / 2.0
            my = (y1 + y2) / 2.0

            # Find occupied cells near this segment (within length/2 radius)
            radius = length / 2.0
            dists_to_mid = np.sqrt(
                (occupied_points[:, 1] - mx) ** 2 + (occupied_points[:, 0] - my) ** 2
            )
            nearby_mask = dists_to_mid <= radius
            nearby = occupied_points[nearby_mask]

            if len(nearby) == 0:
                continue

            # Perpendicular distance from each nearby point to the line
            perp = np.abs(nx * (nearby[:, 1] - x1) + ny * (nearby[:, 0] - y1))
            deviations.append(float(np.mean(perp)))

        if not deviations:
            return 0.5

        mean_dev = float(np.mean(deviations))
        score = 1.0 - mean_dev / max_acceptable
        return float(np.clip(score, 0.0, 1.0))

    def _wall_thickness(self, binary_map, resolution):
        """Score wall thickness consistency.

        Ideal walls are 1 cell thick (distance transform ~1.0 at wall
        centres).
        """
        if np.count_nonzero(binary_map) == 0:
            return 0.5

        # Distance transform on the binary map gives distance from each
        # non-wall cell to the nearest wall.  For wall *thickness* we want
        # the distance from each wall cell to the nearest non-wall cell.
        dist = cv2.distanceTransform(binary_map, cv2.DIST_L2, 5)

        # Sample only at occupied cells
        wall_mask = binary_map == 255
        wall_dists = dist[wall_mask]

        if len(wall_dists) == 0:
            return 0.5

        median_thickness = float(np.median(wall_dists))
        max_thickness = 5.0
        score = 1.0 - abs(median_thickness - 1.0) / max_thickness
        return float(np.clip(score, 0.0, 1.0))

    def _ghost_wall_score(self, binary_map, resolution):
        """Detect ghost / double walls.

        Parallel line pairs with spacing between 2*resolution and
        8*resolution are considered ghost walls.
        """
        lines = cv2.HoughLinesP(
            binary_map,
            rho=1,
            theta=np.pi / 180,
            threshold=20,
            minLineLength=10,
            maxLineGap=5,
        )
        if lines is None or len(lines) == 0:
            return 1.0  # No lines means no ghosts

        total_lines = len(lines)
        ghost_pairs = 0
        angle_threshold = 10.0 * (np.pi / 180.0)
        # Thresholds in cells (pixel coords from HoughLinesP)
        min_dist = 2.0
        max_dist = 10.0

        # Pre-compute angles and midpoints
        angles = []
        midpoints = []
        for line in lines:
            x1, y1, x2, y2 = line[0]
            angle = math.atan2(y2 - y1, x2 - x1)
            angles.append(angle)
            midpoints.append(((x1 + x2) / 2.0, (y1 + y2) / 2.0))

        for i in range(total_lines):
            for j in range(i + 1, total_lines):
                angle_diff = abs(angles[i] - angles[j])
                # Normalise to [0, pi/2]
                angle_diff = min(angle_diff, math.pi - angle_diff)
                if angle_diff > angle_threshold:
                    continue

                dist = math.hypot(
                    midpoints[i][0] - midpoints[j][0],
                    midpoints[i][1] - midpoints[j][1],
                )
                if min_dist <= dist <= max_dist:
                    ghost_pairs += 1

        score = 1.0 - ghost_pairs / max(total_lines, 1)
        return float(np.clip(score, 0.0, 1.0))

    def _symmetry_score(self, occupancy_grid):
        """Measure structural symmetry via PCA and mirrored IoU."""
        known_mask = occupancy_grid != -1
        if np.count_nonzero(known_mask) == 0:
            return 0.5

        occupied_mask = occupancy_grid == 100
        occupied_coords = np.argwhere(occupied_mask).astype(np.float64)
        if len(occupied_coords) < 2:
            return 0.5

        # Centroid of known cells
        known_coords = np.argwhere(known_mask).astype(np.float64)
        centroid = known_coords.mean(axis=0)

        # PCA on occupied cell positions to find principal axis
        centered = occupied_coords - centroid
        cov = np.cov(centered.T)
        eigenvalues, eigenvectors = np.linalg.eigh(cov)
        # Principal axis = eigenvector with largest eigenvalue
        principal_axis = eigenvectors[:, np.argmax(eigenvalues)]

        # Mirror occupied cells across the principal axis through centroid
        # Reflection formula: p' = 2*(p.n)*n - p  (n is unit normal)
        normal = np.array([-principal_axis[1], principal_axis[0]])
        dots = centered @ normal
        mirrored = centered - 2.0 * np.outer(dots, normal)
        mirrored = (mirrored + centroid).astype(np.int32)

        # Build binary images for IoU
        h, w = occupancy_grid.shape
        original_img = np.zeros((h, w), dtype=np.uint8)
        original_img[occupied_mask] = 1

        mirrored_img = np.zeros((h, w), dtype=np.uint8)
        # Clip to valid grid indices
        valid = (
            (mirrored[:, 0] >= 0)
            & (mirrored[:, 0] < h)
            & (mirrored[:, 1] >= 0)
            & (mirrored[:, 1] < w)
        )
        mirrored_valid = mirrored[valid]
        mirrored_img[mirrored_valid[:, 0], mirrored_valid[:, 1]] = 1

        intersection = np.count_nonzero(original_img & mirrored_img)
        union = np.count_nonzero(original_img | mirrored_img)
        if union == 0:
            return 0.5

        return float(intersection / union)

    def _free_space_consistency(self, binary_map, occupancy_grid):
        """Penalize spurious isolated occupied cells in free space."""
        total_occupied = int(np.count_nonzero(binary_map))
        if total_occupied == 0:
            return 1.0

        # Dilate to connect nearby occupied cells (3-cell radius kernel)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        dilated = cv2.dilate(binary_map, kernel, iterations=1)

        # Connected components on dilated image
        num_labels, labels = cv2.connectedComponents(dilated)

        spurious_count = 0
        for label_id in range(1, num_labels):
            component_mask = labels == label_id
            # Count actual occupied cells in this component
            occupied_in_component = int(np.count_nonzero(binary_map[component_mask]))
            if occupied_in_component < 5:
                spurious_count += occupied_in_component

        score = 1.0 - spurious_count / max(total_occupied, 1)
        return float(np.clip(score, 0.0, 1.0))

    def _occupied_density_score(self, binary_map, occupancy_grid):
        """Score whether occupied cell density falls in the expected range.

        Typical indoor maps have 2-10% occupied cells among known cells.
        """
        total_occupied = int(np.count_nonzero(binary_map))
        total_known = int(np.count_nonzero(occupancy_grid != -1))
        if total_known == 0:
            return 0.5

        density = total_occupied / total_known

        if 0.02 <= density <= 0.10:
            return 1.0

        # Linear falloff outside the ideal range, floor at 0.3
        if density < 0.02:
            score = 1.0 - (0.02 - density) / 0.02
        else:
            score = 1.0 - (density - 0.10) / 0.10

        return float(max(score, 0.3))

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def map_to_png(self, map_data, width, height):
        """Convert an occupancy grid to a grayscale PNG image.

        Mapping: unknown (-1) -> 128, free (0) -> 255, occupied (100) -> 0.

        Returns:
            PNG image as bytes.
        """
        grid = np.array(map_data, dtype=np.int8).reshape((height, width))
        image = np.full((height, width), 128, dtype=np.uint8)
        image[grid == 0] = 255
        image[grid == 100] = 0

        success, encoded = cv2.imencode(".png", image)
        if not success:
            raise RuntimeError("Failed to encode map as PNG")
        return encoded.tobytes()

    def compress_map(self, map_data):
        """Compress raw map data using zlib.

        Returns:
            Compressed bytes.
        """
        return zlib.compress(bytes(map_data))
