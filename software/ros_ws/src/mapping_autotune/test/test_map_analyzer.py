"""Tests for map_analyzer module using synthetic occupancy grids."""
import numpy as np
import pytest

from mapping_autotune.map_analyzer import MapAnalyzer


@pytest.fixture
def analyzer():
    return MapAnalyzer()


def _make_empty_grid(width, height, value=0):
    """Create a flat occupancy grid filled with a single value."""
    return [value] * (width * height)


def _make_room_grid(width=100, height=100, wall_thickness=1):
    """Create a simple rectangular room occupancy grid.

    Walls around the perimeter, free space inside, unknown outside is omitted
    (everything is 'known').
    """
    grid = [0] * (width * height)
    for y in range(height):
        for x in range(width):
            # Top and bottom walls
            if y < wall_thickness or y >= height - wall_thickness:
                grid[y * width + x] = 100
            # Left and right walls
            elif x < wall_thickness or x >= width - wall_thickness:
                grid[y * width + x] = 100
    return grid


def _make_smeared_room_grid(width=100, height=100):
    """Create a room with doubled/smeared walls (simulating rotation smear)."""
    grid = [0] * (width * height)
    for y in range(height):
        for x in range(width):
            # Original walls
            if y == 0 or y == height - 1 or x == 0 or x == width - 1:
                grid[y * width + x] = 100
            # Ghost walls offset by 3 cells
            if y == 3 or y == height - 4 or x == 3 or x == width - 4:
                grid[y * width + x] = 100
    return grid


class TestMapAnalyzer:
    def test_analyze_returns_all_metrics(self, analyzer):
        grid = _make_room_grid()
        result = analyzer.analyze(grid, 100, 100, 0.05)
        expected_keys = {
            "wall_straightness", "wall_thickness", "ghost_wall_score",
            "symmetry_score", "free_space_consistency", "occupied_density_score",
            "composite_score", "diagnostics",
        }
        assert expected_keys.issubset(result.keys())

    def test_composite_score_range(self, analyzer):
        grid = _make_room_grid()
        result = analyzer.analyze(grid, 100, 100, 0.05)
        assert 0.0 <= result["composite_score"] <= 1.0

    def test_clean_room_scores_well(self, analyzer):
        """A clean rectangular room should score reasonably well."""
        grid = _make_room_grid()
        result = analyzer.analyze(grid, 100, 100, 0.05)
        assert result["composite_score"] > 0.5

    def test_smeared_room_scores_worse(self, analyzer):
        """A room with ghost walls should score lower than a clean room."""
        clean = _make_room_grid()
        smeared = _make_smeared_room_grid()
        clean_result = analyzer.analyze(clean, 100, 100, 0.05)
        smeared_result = analyzer.analyze(smeared, 100, 100, 0.05)
        assert smeared_result["ghost_wall_score"] < clean_result["ghost_wall_score"]

    def test_empty_grid(self, analyzer):
        """An empty grid (all free) should still return valid scores."""
        grid = _make_empty_grid(50, 50, value=0)
        result = analyzer.analyze(grid, 50, 50, 0.05)
        assert 0.0 <= result["composite_score"] <= 1.0

    def test_unknown_grid(self, analyzer):
        """A fully unknown grid should still return valid scores."""
        grid = _make_empty_grid(50, 50, value=-1)
        result = analyzer.analyze(grid, 50, 50, 0.05)
        assert 0.0 <= result["composite_score"] <= 1.0

    def test_symmetry_on_symmetric_room(self, analyzer):
        """A perfectly symmetric room should have high symmetry score."""
        grid = _make_room_grid(80, 80)
        result = analyzer.analyze(grid, 80, 80, 0.05)
        assert result["symmetry_score"] > 0.6

    def test_map_to_png(self, analyzer):
        grid = _make_room_grid(50, 50)
        png = analyzer.map_to_png(grid, 50, 50)
        assert isinstance(png, bytes)
        assert len(png) > 0
        # PNG magic bytes
        assert png[:4] == b"\x89PNG"

    def test_compress_map(self, analyzer):
        grid = _make_room_grid(50, 50)
        compressed = analyzer.compress_map(grid)
        assert isinstance(compressed, bytes)
        assert len(compressed) < len(grid) * 4  # Should compress well

    def test_free_space_consistency_with_noise(self, analyzer):
        """Scattered random occupied cells in free space should lower the score."""
        grid = _make_room_grid(100, 100)
        # Add random noise in the interior
        rng = np.random.RandomState(42)
        for _ in range(50):
            x = rng.randint(10, 90)
            y = rng.randint(10, 90)
            grid[y * 100 + x] = 100
        result = analyzer.analyze(grid, 100, 100, 0.05)
        clean_result = analyzer.analyze(_make_room_grid(100, 100), 100, 100, 0.05)
        assert result["free_space_consistency"] <= clean_result["free_space_consistency"]
