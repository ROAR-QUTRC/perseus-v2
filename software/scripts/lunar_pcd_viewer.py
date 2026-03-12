#!/usr/bin/env python3
"""
Perseus Lunar PCD Viewer — NASA-style Mission Control Dashboard

Visualises LiDAR point cloud scans (.pcd) as if monitoring a lunar rover
near Shackleton Crater on the Moon's South Pole.

Panels (toggleable via sidebar):
  1. Rotating 3D terrain view (~35 deg elevation, interactive mouse control)
  2. Top-down colour-coded elevation heat map
  3. Topographic contour map with annotated elevation lines
  4. Solar illumination / shadow map for a user-selected date & time
  5. Terrain hazard heatmap (slope + roughness risk)
  6. Path planning preview (click two points for safe A* route)
  7. Line-of-sight comms check (coverage from base station)
  8. Resource overlay (ice deposits + drill sites)
  9. Rover footprint simulation (wheel contact + clearance)
 10. Solar uptime / permanently shadowed regions
 11. Mission suitability score (0.25m grid cells)

Features:
  - Night/day cycle playback (28 Earth-day lunar day)
  - Multi-layer toggle sidebar (up to 4 panels visible)
  - Light / dark theme toggle
  - Mission score hover widget with component breakdown

Usage:
    python3 lunar_pcd_viewer.py <path_to.pcd>
    python3 lunar_pcd_viewer.py scan_1.pcd --port 8060
"""

import argparse
import math
from datetime import datetime, timedelta, timezone

import numpy as np
from scipy.interpolate import griddata
from scipy.ndimage import gaussian_filter, maximum_filter, uniform_filter, zoom
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import dijkstra as sp_dijkstra

import plotly.graph_objects as go
from dash import Dash, html, dcc, Input, Output, State, no_update, ctx
from dash.exceptions import PreventUpdate


# ---------------------------------------------------------------------------
# PCD loader (numpy-fast path for binary float32 PCD v0.7)
# ---------------------------------------------------------------------------


def load_pcd(path: str):
    """Return (N,3) float32 XYZ and (N,) float32 intensity from a .pcd file."""
    with open(path, "rb") as f:
        header_lines = []
        while True:
            line = f.readline().decode("ascii", errors="replace").strip()
            header_lines.append(line)
            if line.startswith("DATA"):
                break

        meta: dict[str, list[str]] = {}
        for hl in header_lines:
            parts = hl.split()
            if len(parts) >= 2:
                meta[parts[0]] = parts[1:]

        fields = meta.get("FIELDS", [])
        n_points = int(meta.get("POINTS", ["0"])[0])
        data_type = meta.get("DATA", ["ascii"])[0].lower()

        if data_type == "binary":
            dt = np.dtype([(fields[i], np.float32) for i in range(len(fields))])
            arr = np.frombuffer(
                f.read(n_points * dt.itemsize), dtype=dt, count=n_points
            )
            points = np.column_stack([arr["x"], arr["y"], arr["z"]]).astype(np.float32)
            intensity = (
                arr["intensity"].astype(np.float32)
                if "intensity" in fields
                else np.zeros(n_points, np.float32)
            )
        else:
            data = np.loadtxt(f, dtype=np.float32, max_rows=n_points)
            xi = fields.index("x") if "x" in fields else 0
            yi = fields.index("y") if "y" in fields else 1
            zi = fields.index("z") if "z" in fields else 2
            points = data[:, [xi, yi, zi]]
            intensity = (
                data[:, fields.index("intensity")]
                if "intensity" in fields
                else np.zeros(n_points, np.float32)
            )

    return points, intensity


# ---------------------------------------------------------------------------
# Terrain grid interpolation
# ---------------------------------------------------------------------------


def make_terrain_grid(points, resolution=250):
    """Interpolate point cloud onto a regular grid for contour / surface plots."""
    x, y, z = points[:, 0], points[:, 1], points[:, 2]
    xi = np.linspace(x.min(), x.max(), resolution)
    yi = np.linspace(y.min(), y.max(), resolution)
    xg, yg = np.meshgrid(xi, yi)

    zg = griddata((x, y), z, (xg, yg), method="linear")
    mask = np.isnan(zg)
    if mask.any():
        zg[mask] = griddata((x, y), z, (xg[mask], yg[mask]), method="nearest")

    zg = gaussian_filter(zg, sigma=1.0)
    return xg, yg, zg


# ---------------------------------------------------------------------------
# Solar illumination model
# ---------------------------------------------------------------------------

# Default: Shackleton Crater rim, lunar South Pole
DEFAULT_LAT = -89.9  # degrees selenographic latitude
DEFAULT_LON = 0.0  # degrees selenographic longitude


def compute_sun_direction(
    dt: datetime, lat_deg: float = DEFAULT_LAT, lon_deg: float = DEFAULT_LON
) -> np.ndarray:
    """Sun direction as a unit vector [east, north, up] in a local tangent frame."""
    if dt.tzinfo is None:
        dt = dt.replace(tzinfo=timezone.utc)

    ref_new_moon = datetime(2000, 1, 6, 18, 14, tzinfo=timezone.utc)
    days_since = (dt - ref_new_moon).total_seconds() / 86400.0

    synodic = 29.530588
    phase = (days_since % synodic) / synodic
    subsolar_lon = (phase * 360.0) % 360.0

    nodical = 27.2122
    lib_phase = (days_since % nodical) / nodical
    moon_tilt = 1.5427
    subsolar_lat = moon_tilt * math.sin(2 * math.pi * lib_phase)

    obs_lat = math.radians(lat_deg)
    obs_lon = math.radians(lon_deg)
    ss_lat = math.radians(subsolar_lat)
    ss_lon = math.radians(subsolar_lon)

    dlon = ss_lon - obs_lon
    cos_c = math.sin(obs_lat) * math.sin(ss_lat) + math.cos(obs_lat) * math.cos(
        ss_lat
    ) * math.cos(dlon)
    cos_c = max(-1.0, min(1.0, cos_c))

    elevation = math.asin(cos_c)

    sin_c = math.sqrt(1 - cos_c * cos_c) + 1e-12
    cos_az = (math.sin(ss_lat) - math.sin(obs_lat) * cos_c) / (
        math.cos(obs_lat) * sin_c + 1e-12
    )
    sin_az = math.cos(ss_lat) * math.sin(dlon) / sin_c
    azimuth = math.atan2(sin_az, cos_az)

    ce = math.cos(elevation)
    return np.array(
        [
            ce * math.sin(azimuth),
            ce * math.cos(azimuth),
            math.sin(elevation),
        ]
    )


def compute_shadow_map(x_grid, y_grid, z_grid, sun_dir):
    """Ray-march shadow map.  Returns (rows, cols) float32 in [0, 1]."""
    rows, cols = z_grid.shape
    if sun_dir[2] <= 0:
        return np.zeros((rows, cols), dtype=np.float32)

    dx = x_grid[0, 1] - x_grid[0, 0] if cols > 1 else 1.0
    dy = y_grid[1, 0] - y_grid[0, 0] if rows > 1 else 1.0
    step = min(abs(dx), abs(dy))

    illum = np.ones((rows, cols), dtype=np.float32)

    for s in range(1, min(int(math.sqrt(rows**2 + cols**2)), 150)):
        sc = int(round(-sun_dir[0] * s * step / dx))
        sr = int(round(-sun_dir[1] * s * step / dy))
        dz = -sun_dir[2] * s * step

        if abs(sr) >= rows or abs(sc) >= cols:
            break

        r0, r1 = max(0, sr), min(rows, rows + sr)
        c0, c1 = max(0, sc), min(cols, cols + sc)
        dr0, dr1 = max(0, -sr), max(0, -sr) + (r1 - r0)
        dc0, dc1 = max(0, -sc), max(0, -sc) + (c1 - c0)
        if r0 >= r1 or c0 >= c1:
            continue

        blocked = z_grid[r0:r1, c0:c1] > z_grid[dr0:dr1, dc0:dc1] + dz
        illum[dr0:dr1, dc0:dc1] = np.where(blocked, 0.0, illum[dr0:dr1, dc0:dc1])

    return illum


# ---------------------------------------------------------------------------
# Terrain analysis — slope, hazard, resources, comms, rover, mission score
# ---------------------------------------------------------------------------


def compute_slope_map(xg, yg, zg):
    """Compute slope angle (degrees) and local elevation variance."""
    dx = xg[0, 1] - xg[0, 0] if xg.shape[1] > 1 else 1.0
    dy = yg[1, 0] - yg[0, 0] if yg.shape[0] > 1 else 1.0

    dzdx = np.gradient(zg, dx, axis=1)
    dzdy = np.gradient(zg, dy, axis=0)

    slope_rad = np.arctan(np.sqrt(dzdx**2 + dzdy**2))
    slope_deg = np.degrees(slope_rad)

    mean_z = uniform_filter(zg, size=5, mode="nearest")
    mean_z2 = uniform_filter(zg**2, size=5, mode="nearest")
    variance = np.maximum(mean_z2 - mean_z**2, 0.0)

    return slope_deg, variance


def compute_hazard_map(slope_deg, variance):
    """Combine slope and variance into hazard score [0, 1]."""
    var_95 = np.percentile(variance, 95)
    norm_var = np.clip(variance / (var_95 + 1e-12), 0.0, 1.0)
    slope_component = np.clip(slope_deg / 15.0, 0.0, 1.0)
    return (slope_component * 0.7 + norm_var * 0.3).astype(np.float64)


def generate_ice_deposits(xg, yg, zg, illum):
    """Estimate water-ice probability and identify candidate drill sites."""
    z_min, z_max = zg.min(), zg.max()
    norm_elev = (zg - z_min) / (z_max - z_min + 1e-12)

    ice_prob = (1.0 - norm_elev) * (1.0 - illum)

    rng = np.random.RandomState(42)
    noise = rng.normal(0, 0.05, size=ice_prob.shape)
    ice_prob = gaussian_filter(ice_prob + noise, sigma=3.0)
    ice_prob = np.clip(ice_prob, 0.0, 1.0)

    threshold = np.percentile(ice_prob, 90)
    local_max = maximum_filter(ice_prob, size=11)
    is_peak = (ice_prob == local_max) & (ice_prob > threshold)

    peak_rows, peak_cols = np.where(is_peak)
    drill_sites = [
        (float(xg[r, c]), float(yg[r, c])) for r, c in zip(peak_rows, peak_cols)
    ]

    return ice_prob, drill_sites


def _bilinear_interp(zg, xg, yg, x, y):
    """Bilinear interpolation of zg at world coordinates (x, y)."""
    rows, cols = zg.shape
    x_min, x_max = float(xg[0, 0]), float(xg[0, -1])
    y_min, y_max = float(yg[0, 0]), float(yg[-1, 0])

    col_f = (x - x_min) / (x_max - x_min + 1e-12) * (cols - 1)
    row_f = (y - y_min) / (y_max - y_min + 1e-12) * (rows - 1)

    is_scalar = np.ndim(col_f) == 0
    col_f = np.atleast_1d(np.clip(np.asarray(col_f, dtype=np.float64), 0, cols - 1.001))
    row_f = np.atleast_1d(np.clip(np.asarray(row_f, dtype=np.float64), 0, rows - 1.001))

    c0 = np.floor(col_f).astype(int)
    r0 = np.floor(row_f).astype(int)
    c1 = np.minimum(c0 + 1, cols - 1)
    r1 = np.minimum(r0 + 1, rows - 1)
    dc = col_f - c0
    dr = row_f - r0

    result = (
        zg[r0, c0] * (1 - dr) * (1 - dc)
        + zg[r0, c1] * (1 - dr) * dc
        + zg[r1, c0] * dr * (1 - dc)
        + zg[r1, c1] * dr * dc
    )

    return float(result[0]) if is_scalar else result


def compute_line_of_sight(xg, yg, zg, p1, p2, antenna_height=1.0):
    """Ray-trace LOS between two world-coordinate points on the terrain."""
    n_samples = 200
    t = np.linspace(0.0, 1.0, n_samples)
    xs = p1[0] + t * (p2[0] - p1[0])
    ys = p1[1] + t * (p2[1] - p1[1])

    profile_dist = np.sqrt((xs - p1[0]) ** 2 + (ys - p1[1]) ** 2)
    profile_z = _bilinear_interp(zg, xg, yg, xs, ys)

    z_start = profile_z[0] + antenna_height
    z_end = profile_z[-1] + antenna_height
    los_z = z_start + t * (z_end - z_start)

    blocked_mask = profile_z > los_z
    blocked_mask[0] = False
    blocked_mask[-1] = False

    return {
        "visible": not np.any(blocked_mask),
        "profile_dist": profile_dist,
        "profile_z": profile_z,
        "los_z": los_z,
        "blocked_mask": blocked_mask,
    }


def compute_comms_coverage(xg, yg, zg, base_pos, antenna_height=1.0):
    """Radial viewshed with RF shadow propagation from a base station.

    Casts rays radially outward from the base in all directions.  Along each
    ray the maximum terrain elevation angle is tracked; any cell whose
    elevation angle falls below the running maximum is in RF shadow.

    Signal strength attenuates with distance (bright near base, dimmer far
    away) and drops to zero behind terrain obstructions, producing clearly
    visible shadow cones that extend outward from obstacles.
    """
    rows, cols = zg.shape
    x_min, x_max = float(xg[0, 0]), float(xg[0, -1])
    y_min, y_max = float(yg[0, 0]), float(yg[-1, 0])
    cell_dx = (x_max - x_min) / (cols - 1) if cols > 1 else 1.0
    cell_dy = (y_max - y_min) / (rows - 1) if rows > 1 else 1.0
    step_size = min(cell_dx, cell_dy) * 0.5  # half-cell steps for accuracy

    base_z = (
        float(_bilinear_interp(zg, xg, yg, base_pos[0], base_pos[1])) + antenna_height
    )
    max_dist = float(np.sqrt((x_max - x_min) ** 2 + (y_max - y_min) ** 2))

    # Cast 720 rays (0.5-deg spacing) outward from the base
    n_rays = 720
    n_steps = int(max_dist / step_size) + 1
    angles = np.linspace(0, 2 * np.pi, n_rays, endpoint=False)
    cos_a = np.cos(angles)
    sin_a = np.sin(angles)

    # coverage stores best signal seen at each cell (-1 = not yet visited)
    coverage = np.full((rows, cols), -1.0, dtype=np.float64)
    max_elev = np.full(n_rays, -np.inf)

    for si in range(1, n_steps):
        r = si * step_size

        # Ray sample positions for all rays at this radius
        rx = base_pos[0] + r * cos_a
        ry = base_pos[1] + r * sin_a

        in_bounds = (rx >= x_min) & (rx <= x_max) & (ry >= y_min) & (ry <= y_max)
        if not in_bounds.any():
            break

        # Terrain height at each ray sample
        rz = np.asarray(_bilinear_interp(zg, xg, yg, rx, ry), dtype=np.float64)

        # Elevation angle from base antenna to terrain surface
        elev = np.arctan2(rz - base_z, r)

        # A cell is in LOS if its elevation angle >= max seen along this ray
        in_los = elev >= max_elev

        # Only update max_elev for rays still inside the grid
        max_elev = np.where(in_bounds, np.maximum(max_elev, elev), max_elev)

        # Signal model: distance-attenuated for LOS, zero for shadow
        dist_atten = 1.0 - 0.5 * (r / max_dist)
        signal = np.where(in_los & in_bounds, dist_atten, 0.0)

        # Map world coords → grid indices
        ci = np.clip(
            np.round((rx - x_min) / (x_max - x_min + 1e-12) * (cols - 1)).astype(int),
            0,
            cols - 1,
        )
        ri = np.clip(
            np.round((ry - y_min) / (y_max - y_min + 1e-12) * (rows - 1)).astype(int),
            0,
            rows - 1,
        )

        # Scatter-write: keep max signal at each cell (if LOS from any ray)
        ib = np.where(in_bounds)[0]
        flat_idx = ri[ib] * cols + ci[ib]
        np.maximum.at(coverage.ravel(), flat_idx, signal[ib])

    # Cells never hit by any ray → zero coverage
    gaps = coverage < 0
    coverage[gaps] = 0.0

    # Fill sparse gaps at grid edges via Gaussian blur of neighbours
    if gaps.any():
        blurred = gaussian_filter(coverage, sigma=1.5)
        coverage[gaps] = blurred[gaps]

    # Base station cell = full signal
    br, bc = _world_to_grid(base_pos, xg, yg)
    coverage[br, bc] = 1.0

    return coverage


def compute_lunar_cycle_illumination(
    xg, yg, zg, start_dt, lat_deg, lon_deg, n_steps=56
):
    """Compute shadow maps and solar radiation across a full 28-day lunar day.

    For each timestep the irradiance on a horizontal surface is
    ``solar_constant * sin(sun_elevation)`` (i.e. ``sun_dir[2]``), gated by
    the binary shadow mask.  This gives a smooth heatmap driven entirely by
    how long each cell is illuminated and how high the sun is at the time,
    without noisy per-cell surface-normal effects.

    Returns
    -------
    illum_stack : (n_steps, rows, cols) float32  — binary illumination per step
    timestamps : list of datetime
    solar_radiation : (rows, cols) float64 — cumulative radiation (Wh/m²)
    psr_mask : (rows, cols) bool — permanently shadowed regions (never lit)
    """
    rows, cols = zg.shape
    dt_step = timedelta(days=28.0 / n_steps)
    hours_per_step = 28.0 * 24.0 / n_steps  # hours each step represents

    illum_stack = np.zeros((n_steps, rows, cols), dtype=np.float32)
    timestamps = []

    # Solar constant at lunar surface ~1361 W/m²
    SOLAR_CONSTANT = 1361.0

    # Accumulate total radiation (W·h/m²)
    solar_radiation = np.zeros((rows, cols), dtype=np.float64)

    for i in range(n_steps):
        t = start_dt + i * dt_step
        timestamps.append(t)
        sun_dir = compute_sun_direction(t, lat_deg, lon_deg)
        shadow = compute_shadow_map(xg, yg, zg, sun_dir)
        illum_stack[i] = shadow

        # Irradiance on a horizontal surface = solar_constant * sin(elevation)
        # sun_dir[2] = sin(elevation); clamp to 0 when sun is below horizon
        sin_elev = max(0.0, float(sun_dir[2]))
        irradiance = SOLAR_CONSTANT * sin_elev  # W/m² (scalar)
        solar_radiation += irradiance * shadow * hours_per_step  # Wh/m²

        if (i + 1) % 7 == 0 or i == 0:
            print(f"[PERSEUS]        Shadow map {i + 1}/{n_steps}")

    solar_uptime = np.mean(illum_stack, axis=0)
    psr_mask = solar_uptime == 0.0

    return illum_stack, timestamps, solar_uptime, solar_radiation, psr_mask


def compute_traversal_cost(slope_deg, hazard, solar_uptime, comms_coverage):
    """Per-cell traversal cost.  Slope > 15 deg is impassable."""
    cost = np.full_like(slope_deg, np.inf, dtype=np.float64)
    passable = slope_deg <= 15.0
    cost[passable] = np.clip(
        np.exp(slope_deg[passable] / 10.0)
        + hazard[passable] * 5.0
        - solar_uptime[passable] * 2.0
        + (1.0 - comms_coverage[passable]) * 3.0,
        0.1,
        None,
    )
    return cost


def _world_to_grid(xy, xg, yg):
    """Convert world (x,y) to grid (row, col)."""
    rows, cols = xg.shape
    x_min, x_max = float(xg[0, 0]), float(xg[0, -1])
    y_min, y_max = float(yg[0, 0]), float(yg[-1, 0])
    c = int(round((xy[0] - x_min) / (x_max - x_min + 1e-12) * (cols - 1)))
    r = int(round((xy[1] - y_min) / (y_max - y_min + 1e-12) * (rows - 1)))
    return (int(np.clip(r, 0, rows - 1)), int(np.clip(c, 0, cols - 1)))


def build_sparse_graph(cost_grid, xg, yg):
    """Build a CSR sparse adjacency matrix for 8-connected grid.

    Edge weight from node i to neighbour j = cost_grid[j] * distance.
    Uses scipy.sparse for C-speed Dijkstra later.

    Returns
    -------
    graph : csr_matrix (N x N) where N = rows * cols
    """
    rows, cols = cost_grid.shape
    x_min, x_max = float(xg[0, 0]), float(xg[0, -1])
    y_min, y_max = float(yg[0, 0]), float(yg[-1, 0])
    cell_dx = (x_max - x_min) / (cols - 1) if cols > 1 else 1.0
    cell_dy = (y_max - y_min) / (rows - 1) if rows > 1 else 1.0

    N = rows * cols

    # Pre-compute neighbour offsets and distances
    offsets = []
    for dr in (-1, 0, 1):
        for dc in (-1, 0, 1):
            if dr == 0 and dc == 0:
                continue
            dist = np.sqrt((dr * cell_dy) ** 2 + (dc * cell_dx) ** 2)
            offsets.append((dr, dc, dist))

    # Vectorised edge construction
    r_idx = np.arange(rows)
    c_idx = np.arange(cols)
    rr, cc = np.meshgrid(r_idx, c_idx, indexing="ij")
    rr_flat = rr.ravel()
    cc_flat = cc.ravel()

    src_list = []
    dst_list = []
    wt_list = []

    for dr, dc, dist in offsets:
        nr = rr_flat + dr
        nc = cc_flat + dc
        valid = (nr >= 0) & (nr < rows) & (nc >= 0) & (nc < cols)
        nr_v = nr[valid]
        nc_v = nc[valid]
        src_v = rr_flat[valid] * cols + cc_flat[valid]
        dst_v = nr_v * cols + nc_v
        # Edge weight = destination cost * distance
        dest_cost = cost_grid[nr_v, nc_v]
        finite_mask = np.isfinite(dest_cost)
        src_list.append(src_v[finite_mask])
        dst_list.append(dst_v[finite_mask])
        wt_list.append(dest_cost[finite_mask] * dist)

    src_all = np.concatenate(src_list)
    dst_all = np.concatenate(dst_list)
    wt_all = np.concatenate(wt_list)

    graph = csr_matrix((wt_all, (src_all, dst_all)), shape=(N, N))
    return graph


def find_path(sparse_graph, cost_grid, xg, yg, start_xy, end_xy):
    """Shortest path using scipy.sparse.csgraph.dijkstra with predecessors."""
    rows, cols = cost_grid.shape
    start = _world_to_grid(start_xy, xg, yg)
    goal = _world_to_grid(end_xy, xg, yg)

    if np.isinf(cost_grid[start[0], start[1]]) or np.isinf(cost_grid[goal[0], goal[1]]):
        return None, None

    src_node = start[0] * cols + start[1]
    dst_node = goal[0] * cols + goal[1]

    dist_matrix, predecessors = sp_dijkstra(
        sparse_graph,
        directed=True,
        indices=src_node,
        return_predecessors=True,
        limit=np.inf,
    )

    total_cost = dist_matrix[dst_node]
    if np.isinf(total_cost):
        return None, None

    # Trace path from goal back to start via predecessors
    path_nodes = []
    node = dst_node
    while node != src_node and node >= 0:
        path_nodes.append(node)
        node = predecessors[node]
    if node < 0:
        return None, None
    path_nodes.append(src_node)
    path_nodes.reverse()

    path_coords = []
    for and in path_nodes:
        r, c = divmod(and, cols)
        path_coords.append((float(xg[r, c]), float(yg[r, c])))

    return path_coords, float(total_cost)


def compute_rover_footprint(
    xg,
    yg,
    zg,
    rover_pos,
    rover_heading=0,
    wheelbase=0.6,
    track_width=0.5,
    ground_clearance=0.15,
):
    """Compute rover wheel positions, tilt, and ground clearance at a pose."""
    heading_rad = np.radians(rover_heading)
    cos_h, sin_h = np.cos(heading_rad), np.sin(heading_rad)

    def rotate(dx, dy):
        return (dx * cos_h - dy * sin_h, dx * sin_h + dy * cos_h)

    half_wb, half_tw = wheelbase / 2.0, track_width / 2.0
    wheel_offsets = [
        (half_wb, half_tw),
        (half_wb, -half_tw),
        (-half_wb, half_tw),
        (-half_wb, -half_tw),
    ]

    wheel_positions = np.zeros((4, 3))
    contact_ok = np.zeros(4, dtype=bool)

    for i, (fwd, lat) in enumerate(wheel_offsets):
        dx, dy = rotate(fwd, lat)
        wx, wy = rover_pos[0] + dx, rover_pos[1] + dy
        wz = _bilinear_interp(zg, xg, yg, wx, wy)
        wheel_positions[i] = [wx, wy, float(wz)]
        contact_ok[i] = True

    # Fit plane through wheel contact points
    if contact_ok.sum() >= 3:
        pts = wheel_positions[contact_ok]
        A_mat = np.column_stack([pts[:, 0], pts[:, 1], np.ones(len(pts))])
        plane_coeffs, _, _, _ = np.linalg.lstsq(A_mat, pts[:, 2], rcond=None)
        a, b, _ = plane_coeffs
        tilt_pitch = float(np.degrees(np.arctan2(a * cos_h + b * sin_h, 1.0)))
        tilt_roll = float(np.degrees(np.arctan2(-a * sin_h + b * cos_h, 1.0)))
    else:
        tilt_pitch = tilt_roll = 0.0

    body_corners = np.zeros((4, 2))
    for i, (fwd, lat) in enumerate(wheel_offsets):
        dx, dy = rotate(fwd, lat)
        body_corners[i] = [rover_pos[0] + dx, rover_pos[1] + dy]

    # Min clearance under chassis
    if contact_ok.sum() >= 3:
        chassis_z = np.mean(wheel_positions[contact_ok, 2]) + ground_clearance
        min_clearance = ground_clearance
        for u in np.linspace(-half_wb, half_wb, 5):
            for v in np.linspace(-half_tw, half_tw, 5):
                dx, dy = rotate(u, v)
                bz = _bilinear_interp(zg, xg, yg, rover_pos[0] + dx, rover_pos[1] + dy)
                min_clearance = min(min_clearance, chassis_z - float(bz))
    else:
        min_clearance = ground_clearance

    return {
        "wheel_positions": wheel_positions,
        "contact_ok": contact_ok,
        "min_clearance": float(min_clearance),
        "tilt_pitch": tilt_pitch,
        "tilt_roll": tilt_roll,
        "body_corners": body_corners,
    }


def compute_mission_score(
    xg,
    yg,
    zg,
    slope_deg,
    solar_uptime,
    comms_coverage,
    hazard,
    ice_prob,
    cell_size=0.25,
):
    """Divide terrain into cell_size x cell_size cells and score each.

    Uses scipy.ndimage.zoom to downsample each metric to the score grid
    resolution instead of Python loops.
    """
    rows, cols = zg.shape
    x_min, x_max = float(xg[0, 0]), float(xg[0, -1])
    y_min, y_max = float(yg[0, 0]), float(yg[-1, 0])

    score_cols = max(1, int(np.floor((x_max - x_min) / cell_size)))
    score_rows = max(1, int(np.floor((y_max - y_min) / cell_size)))

    score_xi = np.linspace(
        x_min + cell_size / 2, x_min + (score_cols - 0.5) * cell_size, score_cols
    )
    score_yi = np.linspace(
        y_min + cell_size / 2, y_min + (score_rows - 0.5) * cell_size, score_rows
    )
    score_xg_out, score_yg_out = np.meshgrid(score_xi, score_yi)

    # Downsample each metric via zoom (acts as block averaging with order=1)
    def _resample(arr):
        return zoom(
            arr.astype(np.float64), (score_rows / rows, score_cols / cols), order=1
        )

    solar_sc = np.clip(_resample(solar_uptime), 0, 1)
    slope_sc = np.clip(1.0 - _resample(slope_deg) / 30.0, 0, 1)
    rough_sc = np.clip(1.0 - _resample(hazard), 0, 1)
    comms_sc = np.clip(_resample(comms_coverage), 0, 1)

    score_grid = solar_sc * 25.0 + slope_sc * 25.0 + rough_sc * 25.0 + comms_sc * 25.0

    best_idx = np.unravel_index(np.argmax(score_grid), score_grid.shape)
    summary = {
        "mean_score": float(np.mean(score_grid)),
        "max_score": float(np.max(score_grid)),
        "best_location": (float(score_xg_out[best_idx]), float(score_yg_out[best_idx])),
        "coverage_above_70": float(np.mean(score_grid > 70.0) * 100.0),
    }

    components = {
        "solar": solar_sc * 25.0,
        "slope": slope_sc * 25.0,
        "clearance": rough_sc * 25.0,
        "comms": comms_sc * 25.0,
    }

    return score_xg_out, score_yg_out, score_grid, components, summary


# ---------------------------------------------------------------------------
# Battery / energy model
# ---------------------------------------------------------------------------

# Rover electrical specs
BATTERY_VOLTAGE = 24.0  # V
BATTERY_CAPACITY = 50.0  # Ah
BATTERY_ENERGY_WH = BATTERY_VOLTAGE * BATTERY_CAPACITY  # 1200 Wh total

CURRENT_IDLE = 0.5  # A — stationary
CURRENT_FLAT = 4.0  # A — driving on flat ground
CURRENT_MAX = 40.0  # A — max draw in rough/steep terrain

# Rover speed assumption for energy-per-metre calculation
ROVER_SPEED_MS = 0.3  # m/s typical lunar rover crawl


def compute_energy_cost_grid(slope_deg, hazard):
    """Compute energy cost in Wh to traverse each grid cell.

    Current draw is interpolated between CURRENT_FLAT (slope=0, hazard=0)
    and CURRENT_MAX (slope>=15 or hazard>=1).  Slope > 20 deg is impassable.

    Energy per cell = voltage * current * (cell_size / speed).
    Cell size is implicitly 1 grid spacing; the caller scales by actual distance.

    Returns
    -------
    wh_per_metre : ndarray (rows, cols)
        Energy cost in Wh per metre of travel.  np.inf for impassable.
    """
    # Terrain difficulty factor [0, 1]: 0 = easy flat, 1 = worst passable
    difficulty = np.clip(np.maximum(slope_deg / 15.0, hazard), 0.0, 1.0)

    # Current draw: linear interpolation from flat to max
    current = CURRENT_FLAT + difficulty * (CURRENT_MAX - CURRENT_FLAT)

    # Power = V * I, energy per metre = power / speed
    power_w = BATTERY_VOLTAGE * current
    wh_per_metre = power_w / ROVER_SPEED_MS / 3600.0  # W / (m/s) = J/m, /3600 = Wh/m

    # Impassable cells
    wh_per_metre[slope_deg > 20.0] = np.inf

    return wh_per_metre


def compute_battery_range(
    energy_graph, rows, cols, xg, yg, lander_pos, charge_pct=100.0
):
    """Dijkstra from lander using scipy.sparse.csgraph for C-speed.

    Parameters
    ----------
    energy_graph : csr_matrix (N x N)
        Pre-built sparse graph with energy cost weights (Wh).
    rows, cols : int
        Grid dimensions.
    xg, yg : ndarray (rows, cols)
        Meshgrid coordinate arrays.
    lander_pos : tuple (x, y)
        Lander / base station position in world coordinates.
    charge_pct : float
        Current battery charge percentage (0-100).

    Returns
    -------
    cost_to_reach : ndarray (rows, cols)
        Min energy in Wh to reach each cell from the lander.  inf if unreachable.
    reachable : ndarray (rows, cols) bool
        True if rover can reach the cell AND return with >= 25% battery.
    range_pct : ndarray (rows, cols)
        Remaining battery % after round-trip to each cell.  Negative = out of range.
    """
    lr, lc = _world_to_grid(lander_pos, xg, yg)
    src_node = lr * cols + lc

    # Available energy (Wh)
    total_wh = BATTERY_ENERGY_WH * (charge_pct / 100.0)
    reserve_wh = BATTERY_ENERGY_WH * 0.25

    # C-speed Dijkstra — single source, all destinations
    dist_flat = sp_dijkstra(energy_graph, directed=True, indices=src_node)
    cost_to_reach = dist_flat.reshape(rows, cols)

    # Round trip: approximate return cost = same as outbound (symmetric terrain)
    round_trip_wh = cost_to_reach * 2.0

    # Reachable if round trip + reserve fits in available energy
    usable_wh = total_wh - reserve_wh
    reachable = round_trip_wh <= usable_wh
    reachable[np.isinf(cost_to_reach)] = False

    # Remaining battery % after round trip
    range_pct = ((total_wh - round_trip_wh) / BATTERY_ENERGY_WH) * 100.0
    range_pct[np.isinf(cost_to_reach)] = -100.0

    return cost_to_reach, reachable, range_pct


# ---------------------------------------------------------------------------
# Colour scales
# ---------------------------------------------------------------------------

LUNAR_CS = [
    [0.0, "#1a1a2e"],
    [0.15, "#2d2d44"],
    [0.3, "#4a4a5a"],
    [0.5, "#6b6b7b"],
    [0.7, "#8a8a96"],
    [0.85, "#a8a8b0"],
    [1.0, "#c8c8cc"],
]

TOPO_CS = [
    [0.0, "#000033"],
    [0.1, "#000066"],
    [0.2, "#003399"],
    [0.3, "#0066cc"],
    [0.4, "#3399cc"],
    [0.5, "#66cc99"],
    [0.6, "#99cc66"],
    [0.7, "#cccc33"],
    [0.8, "#cc9933"],
    [0.9, "#cc6633"],
    [1.0, "#cc3333"],
]

SHADOW_CS = [
    [0.0, "#000011"],
    [0.2, "#0a0a33"],
    [0.4, "#333355"],
    [0.6, "#888899"],
    [0.8, "#bbbbcc"],
    [1.0, "#eeeef5"],
]

HAZARD_CS = [
    [0.0, "#00ff88"],
    [0.3, "#66cc44"],
    [0.5, "#ccaa00"],
    [0.7, "#ff8800"],
    [0.85, "#ff4400"],
    [1.0, "#cc0000"],
]

COMMS_CS = [
    [0.0, "#cc0000"],
    [0.25, "#ff4400"],
    [0.5, "#ccaa00"],
    [0.75, "#66cc44"],
    [1.0, "#00ff88"],
]

ICE_CS = [
    [0.0, "#000033"],
    [0.25, "#001166"],
    [0.5, "#006688"],
    [0.75, "#00ccdd"],
    [1.0, "#eeffff"],
]

SCORE_CS = [
    [0.0, "#cc0000"],
    [0.25, "#ff4400"],
    [0.5, "#ccaa00"],
    [0.75, "#66cc44"],
    [1.0, "#00ff88"],
]

PSR_CS = [
    [0.0, "#000000"],
    [0.25, "#1a0033"],
    [0.5, "#330066"],
    [0.75, "#660088"],
    [1.0, "#cc00ff"],
]

# Solar radiation: black (none) → dark red → orange → yellow → white (peak)
SOLAR_RAD_CS = [
    [0.0, "#000000"],
    [0.1, "#1a0000"],
    [0.2, "#660000"],
    [0.35, "#cc2200"],
    [0.5, "#ee6600"],
    [0.65, "#ffaa00"],
    [0.8, "#ffdd44"],
    [0.9, "#ffee99"],
    [1.0, "#ffffff"],
]

RANGE_CS = [
    [0.0, "#cc0000"],
    [0.25, "#ff4400"],
    [0.35, "#ff8800"],
    [0.5, "#ccaa00"],
    [0.65, "#88cc00"],
    [0.8, "#44dd44"],
    [1.0, "#00ff88"],
]


# ---------------------------------------------------------------------------
# Theme system — dark and light modes
# ---------------------------------------------------------------------------

THEMES = {
    "dark": dict(
        page_bg="#0a0a12",
        panel_bg="#0d1117",
        grid_color="#1a2332",
        font_color="#00ff88",
        accent="#00ccff",
        muted="#aaaaaa",
        sidebar_bg="#0d1117",
        input_bg="#1a1a2e",
        btn_bg="#1a3322",
    ),
    "light": dict(
        page_bg="#f0f2f5",
        panel_bg="#ffffff",
        grid_color="#cccccc",
        font_color="#003322",
        accent="#0066cc",
        muted="#666666",
        sidebar_bg="#e8eaed",
        input_bg="#ffffff",
        btn_bg="#d4edda",
    ),
}


def _t(theme_name):
    """Resolve theme dict by name."""
    return THEMES.get(theme_name, THEMES["dark"])


def _layout_base(t):
    return dict(
        paper_bgcolor=t["panel_bg"],
        plot_bgcolor=t["panel_bg"],
        font=dict(family="Courier New, monospace", size=11, color=t["font_color"]),
        margin=dict(l=40, r=40, t=45, b=35),
    )


def _xaxis(t):
    return dict(
        title="X (m)",
        scaleanchor="y",
        gridcolor=t["grid_color"],
        color=t["font_color"],
        zerolinecolor=t["grid_color"],
    )


def _yaxis(t):
    return dict(
        title="Y (m)",
        gridcolor=t["grid_color"],
        color=t["font_color"],
        zerolinecolor=t["grid_color"],
    )


def _colorbar(title_text, t):
    return dict(
        title=dict(text=title_text, font=dict(color=t["font_color"], size=11)),
        len=0.45,
        thickness=12,
        tickfont=dict(color=t["font_color"], size=10),
    )


# ---------------------------------------------------------------------------
# Figure builders (pure functions — return go.Figure)
# ---------------------------------------------------------------------------


def fig_3d(points, intensity, subsample, camera_eye, theme="dark"):
    """Build the 3D scatter terrain figure."""
    t = _t(theme)
    pts = points[::subsample]
    z = pts[:, 2]

    fig = go.Figure(
        data=[
            go.Scatter3d(
                x=pts[:, 0],
                y=pts[:, 1],
                z=pts[:, 2],
                mode="markers",
                marker=dict(
                    size=1.2,
                    color=z,
                    colorscale=LUNAR_CS,
                    opacity=0.85,
                    colorbar=dict(
                        title=dict(
                            text="Elev (m)", font=dict(color=t["font_color"], size=11)
                        ),
                        len=0.5,
                        thickness=12,
                        x=0.98,
                        tickfont=dict(color=t["font_color"], size=10),
                    ),
                ),
                hovertemplate="X: %{x:.2f}m<br>Y: %{y:.2f}m<br>Z: %{z:.2f}m<extra></extra>",
            )
        ]
    )
    fig.update_layout(
        **_layout_base(t),
        scene=dict(
            xaxis=dict(
                title="X (m)",
                gridcolor=t["grid_color"],
                showbackground=True,
                backgroundcolor=t["panel_bg"],
                color=t["font_color"],
                zerolinecolor=t["grid_color"],
            ),
            yaxis=dict(
                title="Y (m)",
                gridcolor=t["grid_color"],
                showbackground=True,
                backgroundcolor=t["panel_bg"],
                color=t["font_color"],
                zerolinecolor=t["grid_color"],
            ),
            zaxis=dict(
                title="Z (m)",
                gridcolor=t["grid_color"],
                showbackground=True,
                backgroundcolor=t["panel_bg"],
                color=t["font_color"],
                zerolinecolor=t["grid_color"],
            ),
            camera=dict(
                eye=camera_eye, center=dict(x=0, y=0, z=0), up=dict(x=0, y=0, z=1)
            ),
            aspectmode="data",
        ),
        uirevision="terrain",
    )
    return fig


def fig_heatmap(xg, yg, zg, theme="dark"):
    t = _t(theme)
    fig = go.Figure(
        data=[
            go.Heatmap(
                x=xg[0, :],
                y=yg[:, 0],
                z=zg,
                colorscale=TOPO_CS,
                colorbar=_colorbar("Elev (m)", t),
                hovertemplate="X: %{x:.2f}m<br>Y: %{y:.2f}m<br>Elev: %{z:.2f}m<extra></extra>",
            )
        ]
    )
    fig.update_layout(**_layout_base(t), xaxis=_xaxis(t), yaxis=_yaxis(t))
    return fig


def fig_contour(xg, yg, zg, n_contours=20, theme="dark"):
    t = _t(theme)
    fig = go.Figure(
        data=[
            go.Contour(
                x=xg[0, :],
                y=yg[:, 0],
                z=zg,
                ncontours=n_contours,
                contours=dict(
                    showlabels=True,
                    labelfont=dict(size=9, color=t["accent"]),
                    coloring="heatmap",
                ),
                colorscale=TOPO_CS,
                colorbar=_colorbar("Elev (m)", t),
                line=dict(
                    width=1.5,
                    color=f"rgba({','.join(str(int(t['accent'].lstrip('#')[i : i + 2], 16)) for i in (0, 2, 4))},0.5)",
                ),
                hovertemplate="X: %{x:.2f}m<br>Y: %{y:.2f}m<br>Elev: %{z:.2f}m<extra></extra>",
            )
        ]
    )
    fig.update_layout(**_layout_base(t), xaxis=_xaxis(t), yaxis=_yaxis(t))
    return fig


def fig_shadow(xg, yg, zg, illum, theme="dark"):
    t = _t(theme)
    z_norm = (zg - zg.min()) / (zg.max() - zg.min() + 1e-9)
    shaded = z_norm * (0.3 + 0.7 * illum)
    fig = go.Figure(
        data=[
            go.Heatmap(
                x=xg[0, :],
                y=yg[:, 0],
                z=shaded,
                colorscale=SHADOW_CS,
                showscale=False,
                hovertemplate="X: %{x:.2f}m<br>Y: %{y:.2f}m<extra></extra>",
            )
        ]
    )
    fig.update_layout(**_layout_base(t), xaxis=_xaxis(t), yaxis=_yaxis(t))
    return fig


def fig_hazard(xg, yg, hazard, slope_deg, theme="dark"):
    t = _t(theme)
    custom = np.stack([slope_deg, hazard], axis=-1)
    fig = go.Figure(
        data=[
            go.Heatmap(
                x=xg[0, :],
                y=yg[:, 0],
                z=hazard,
                colorscale=HAZARD_CS,
                colorbar=_colorbar("Hazard", t),
                customdata=custom,
                hovertemplate=(
                    "X: %{x:.2f}m<br>Y: %{y:.2f}m<br>"
                    "Slope: %{customdata[0]:.1f} deg<br>"
                    "Hazard: %{customdata[1]:.2f}<extra></extra>"
                ),
            )
        ]
    )
    fig.update_layout(**_layout_base(t), xaxis=_xaxis(t), yaxis=_yaxis(t))
    return fig


def fig_resources(xg, yg, ice_prob, drill_sites, theme="dark"):
    t = _t(theme)
    fig = go.Figure(
        data=[
            go.Heatmap(
                x=xg[0, :],
                y=yg[:, 0],
                z=ice_prob * 100,
                colorscale=ICE_CS,
                colorbar=_colorbar("Ice %", t),
                hovertemplate="X: %{x:.2f}m<br>Y: %{y:.2f}m<br>Ice: %{z:.1f}%<extra></extra>",
            )
        ]
    )
    if drill_sites and len(drill_sites) > 0:
        ds = np.asarray(drill_sites)
        fig.add_trace(
            go.Scatter(
                x=ds[:, 0],
                y=ds[:, 1],
                mode="markers",
                marker=dict(
                    symbol="star",
                    size=14,
                    color="#ff00ff",
                    line=dict(width=1, color="#ffffff"),
                ),
                name="Drill Sites",
                hovertemplate="DRILL SITE<br>X: %{x:.2f}m<br>Y: %{y:.2f}m<extra></extra>",
            )
        )
    fig.update_layout(**_layout_base(t), xaxis=_xaxis(t), yaxis=_yaxis(t))
    return fig


def fig_comms(
    xg, yg, comms_coverage, base_pos, rover_pos=None, los_data=None, theme="dark"
):
    t = _t(theme)
    # Build custom hover data: coverage %, distance from base
    dx = xg - base_pos[0]
    dy = yg - base_pos[1]
    dist_from_base = np.sqrt(dx**2 + dy**2)
    custom = np.stack([comms_coverage * 100, dist_from_base], axis=-1)

    fig = go.Figure(
        data=[
            go.Heatmap(
                x=xg[0, :],
                y=yg[:, 0],
                z=comms_coverage,
                colorscale=COMMS_CS,
                colorbar=_colorbar("Signal", t),
                customdata=custom,
                hovertemplate=(
                    "X: %{x:.2f}m<br>Y: %{y:.2f}m<br>"
                    "LOS: %{customdata[0]:.0f}%<br>"
                    "Range: %{customdata[1]:.1f}m<extra></extra>"
                ),
            )
        ]
    )

    # Shadow boundary contour — outlines where signal drops into shadow
    fig.add_trace(
        go.Contour(
            x=xg[0, :],
            y=yg[:, 0],
            z=comms_coverage,
            contours=dict(
                start=0.15, end=0.15, size=0.1, coloring="none", showlabels=False
            ),
            line=dict(width=2, color="#ff4400", dash="dot"),
            showscale=False,
            hoverinfo="skip",
            name="Shadow Edge",
        )
    )

    # Base station marker + label
    fig.add_trace(
        go.Scatter(
            x=[base_pos[0]],
            y=[base_pos[1]],
            mode="markers+text",
            marker=dict(
                symbol="diamond",
                size=16,
                color=t["accent"],
                line=dict(width=2, color="#ffffff"),
            ),
            text=["BASE"],
            textposition="top center",
            textfont=dict(size=12, color=t["accent"], family="Courier New"),
            name="Base",
            hovertemplate="BASE STATION<br>X: %{x:.2f}m<br>Y: %{y:.2f}m<extra></extra>",
        )
    )

    if rover_pos is not None:
        fig.add_trace(
            go.Scatter(
                x=[rover_pos[0]],
                y=[rover_pos[1]],
                mode="markers+text",
                marker=dict(
                    symbol="circle",
                    size=12,
                    color=t["font_color"],
                    line=dict(width=1, color="#ffffff"),
                ),
                text=["ROVER"],
                textposition="top center",
                textfont=dict(size=10, color=t["font_color"], family="Courier New"),
                name="Rover",
                hovertemplate="ROVER<br>X: %{x:.2f}m<br>Y: %{y:.2f}m<extra></extra>",
            )
        )
        if los_data is not None and "blocked_mask" in los_data:
            bm = los_data["blocked_mask"]
            n_seg = len(bm)
            lx = np.linspace(base_pos[0], rover_pos[0], n_seg + 1)
            ly = np.linspace(base_pos[1], rover_pos[1], n_seg + 1)
            for i in range(0, n_seg, 3):
                seg_color = "#cc0000" if bm[i] else "#00ff88"
                end = min(i + 4, n_seg)
                fig.add_trace(
                    go.Scatter(
                        x=[lx[i], lx[end]],
                        y=[ly[i], ly[end]],
                        mode="lines",
                        line=dict(width=3, color=seg_color),
                        showlegend=False,
                        hoverinfo="skip",
                    )
                )
    fig.update_layout(**_layout_base(t), xaxis=_xaxis(t), yaxis=_yaxis(t))
    return fig


def fig_path_plan(
    xg,
    yg,
    hazard,
    path_coords=None,
    start_xy=None,
    end_xy=None,
    energy_cost=None,
    theme="dark",
):
    t = _t(theme)
    fig = go.Figure(
        data=[
            go.Heatmap(
                x=xg[0, :],
                y=yg[:, 0],
                z=hazard,
                colorscale=HAZARD_CS,
                opacity=0.4,
                showscale=False,
                hovertemplate="X: %{x:.2f}m<br>Y: %{y:.2f}m<br>Hazard: %{z:.2f}<extra></extra>",
            )
        ]
    )
    if path_coords is not None and len(path_coords) > 0:
        pc = np.asarray(path_coords)
        fig.add_trace(
            go.Scatter(
                x=pc[:, 0],
                y=pc[:, 1],
                mode="lines",
                line=dict(width=3, color=t["accent"]),
                name="Path",
                hovertemplate="Path<br>X: %{x:.2f}m<br>Y: %{y:.2f}m<extra></extra>",
            )
        )
    if start_xy is not None:
        fig.add_trace(
            go.Scatter(
                x=[start_xy[0]],
                y=[start_xy[1]],
                mode="markers",
                marker=dict(
                    symbol="circle",
                    size=14,
                    color="#00ff88",
                    line=dict(width=2, color="#ffffff"),
                ),
                name="Start",
            )
        )
    if end_xy is not None:
        fig.add_trace(
            go.Scatter(
                x=[end_xy[0]],
                y=[end_xy[1]],
                mode="markers",
                marker=dict(
                    symbol="circle",
                    size=14,
                    color="#cc0000",
                    line=dict(width=2, color="#ffffff"),
                ),
                name="End",
            )
        )
    if energy_cost is not None:
        fig.add_annotation(
            text=f"ENERGY: {energy_cost:.1f}",
            xref="paper",
            yref="paper",
            x=0.98,
            y=0.98,
            showarrow=False,
            font=dict(size=13, color=t["accent"], family="Courier New, monospace"),
            bgcolor=t["panel_bg"],
            bordercolor=t["accent"],
            borderwidth=1,
        )
    elif path_coords is None:
        fig.add_annotation(
            text="CLICK TO SET WAYPOINTS",
            xref="paper",
            yref="paper",
            x=0.5,
            y=0.5,
            showarrow=False,
            font=dict(size=16, color=t["accent"], family="Courier New, monospace"),
        )
    fig.update_layout(**_layout_base(t), xaxis=_xaxis(t), yaxis=_yaxis(t))
    return fig


def fig_rover(xg, yg, zg, slope_deg, rover_data=None, theme="dark"):
    t = _t(theme)
    fig = go.Figure(
        data=[
            go.Contour(
                x=xg[0, :],
                y=yg[:, 0],
                z=slope_deg,
                contours=dict(
                    showlabels=True,
                    labelfont=dict(size=9, color=t["font_color"]),
                    coloring="heatmap",
                ),
                colorscale=HAZARD_CS,
                showscale=False,
                line=dict(width=1, color="rgba(0,255,170,0.3)"),
                hovertemplate="X: %{x:.2f}m<br>Y: %{y:.2f}m<br>Slope: %{z:.1f} deg<extra></extra>",
            )
        ]
    )
    if rover_data is not None:
        bc = np.asarray(rover_data["body_corners"])
        bx = list(bc[:, 0]) + [bc[0, 0]]
        by = list(bc[:, 1]) + [bc[0, 1]]
        fig.add_trace(
            go.Scatter(
                x=bx,
                y=by,
                mode="lines",
                fill="toself",
                fillcolor="rgba(0,204,255,0.25)",
                line=dict(width=2, color=t["accent"]),
                name="Body",
                hoverinfo="skip",
            )
        )
        wp = np.asarray(rover_data["wheel_positions"])
        contact = rover_data["contact_ok"]
        w_colors = ["#00ff88" if c else "#cc0000" for c in contact]
        fig.add_trace(
            go.Scatter(
                x=wp[:, 0],
                y=wp[:, 1],
                mode="markers",
                marker=dict(
                    symbol="circle",
                    size=10,
                    color=w_colors,
                    line=dict(width=1, color="#ffffff"),
                ),
                name="Wheels",
            )
        )
        cx = float(np.mean(bc[:, 0]))
        cy = float(np.mean(bc[:, 1]))
        clr = rover_data["min_clearance"]
        clr_color = "#00ff88" if clr > 0.05 else "#ffaa00" if clr > 0 else "#cc0000"
        fig.add_annotation(
            text=f"CLR: {clr:.2f}m | P: {rover_data['tilt_pitch']:.1f} deg "
            f"R: {rover_data['tilt_roll']:.1f} deg",
            x=cx,
            y=cy,
            showarrow=False,
            font=dict(size=10, color=clr_color, family="Courier New, monospace"),
            yshift=22,
            bgcolor=t["panel_bg"],
            bordercolor=clr_color,
            borderwidth=1,
        )
    else:
        fig.add_annotation(
            text="CLICK TO PLACE ROVER",
            xref="paper",
            yref="paper",
            x=0.5,
            y=0.5,
            showarrow=False,
            font=dict(size=16, color=t["accent"], family="Courier New, monospace"),
        )
    fig.update_layout(**_layout_base(t), xaxis=_xaxis(t), yaxis=_yaxis(t))
    return fig


def fig_psr(xg, yg, solar_radiation, psr_mask, theme="dark"):
    """Solar radiation heatmap over a full lunar day (28 Earth days).

    Shows cumulative solar energy received per grid cell in kWh/m²,
    accounting for sun elevation, surface slope, and terrain shadowing.
    """
    t = _t(theme)
    rad_kwh = solar_radiation / 1000.0  # Wh/m² → kWh/m²
    rad_max = float(np.max(rad_kwh)) if np.any(rad_kwh > 0) else 1.0

    custom = np.stack(
        [
            solar_radiation,  # Wh/m²
            (solar_radiation / (rad_max * 1000.0 + 1e-9)) * 100,  # % of max
        ],
        axis=-1,
    )

    fig = go.Figure(
        data=[
            go.Heatmap(
                x=xg[0, :],
                y=yg[:, 0],
                z=rad_kwh,
                colorscale=SOLAR_RAD_CS,
                colorbar=_colorbar("kWh/m²", t),
                customdata=custom,
                hovertemplate=(
                    "X: %{x:.2f}m<br>Y: %{y:.2f}m<br>"
                    "Radiation: %{z:.1f} kWh/m²<br>"
                    "%{customdata[1]:.0f}% of peak<extra></extra>"
                ),
            )
        ]
    )

    # PSR boundary contour
    psr_float = psr_mask.astype(np.float32)
    fig.add_trace(
        go.Contour(
            x=xg[0, :],
            y=yg[:, 0],
            z=psr_float,
            contours=dict(start=0.5, end=0.5, size=1, coloring="none"),
            line=dict(width=2, color="#ff00ff"),
            showscale=False,
            hoverinfo="skip",
            name="PSR Boundary",
        )
    )

    psr_count = int(np.sum(psr_mask))
    psr_pct = psr_count / (xg.shape[0] * xg.shape[1]) * 100
    fig.add_annotation(
        text=(
            f"PSR: {psr_count:,} cells ({psr_pct:.1f}%) | "
            f"Peak: {rad_max:.0f} kWh/m² | "
            f"Mean: {float(np.mean(rad_kwh)):.0f} kWh/m²"
        ),
        xref="paper",
        yref="paper",
        x=0.98,
        y=0.98,
        showarrow=False,
        font=dict(size=11, color="#ff00ff", family="Courier New, monospace"),
        bgcolor=t["panel_bg"],
        bordercolor="#ff00ff",
        borderwidth=1,
    )
    fig.update_layout(**_layout_base(t), xaxis=_xaxis(t), yaxis=_yaxis(t))
    return fig


def fig_mission_score(score_xg, score_yg, score_grid, components, theme="dark"):
    t = _t(theme)
    custom = np.stack(
        [
            components["solar"],
            components["slope"],
            components["clearance"],
            components["comms"],
            score_grid,
        ],
        axis=-1,
    )
    fig = go.Figure(
        data=[
            go.Heatmap(
                x=score_xg[0, :],
                y=score_yg[:, 0],
                z=score_grid,
                colorscale=SCORE_CS,
                zmin=0,
                zmax=100,
                colorbar=_colorbar("Score", t),
                customdata=custom,
                hovertemplate=(
                    "X: %{x:.2f}m  Y: %{y:.2f}m<br>"
                    "Solar: %{customdata[0]:.0f}/25 | "
                    "Slope: %{customdata[1]:.0f}/25<br>"
                    "Clear: %{customdata[2]:.0f}/25 | "
                    "Comms: %{customdata[3]:.0f}/25<br>"
                    "TOTAL: %{customdata[4]:.0f}/100<extra></extra>"
                ),
            )
        ]
    )
    fig.update_layout(**_layout_base(t), xaxis=_xaxis(t), yaxis=_yaxis(t))
    return fig


def fig_battery_range(
    xg,
    yg,
    range_pct,
    reachable,
    lander_pos,
    waypoints=None,
    wp_path=None,
    wp_cost=None,
    charge_pct=100.0,
    theme="dark",
):
    """Battery range heatmap with lander, waypoints, and planned route."""
    t = _t(theme)

    # Clamp display range to [0, 100] for colour mapping
    display_z = np.clip(range_pct, 0, 100)
    # Grey out unreachable cells
    display_z[~reachable] = np.nan

    fig = go.Figure(
        data=[
            go.Heatmap(
                x=xg[0, :],
                y=yg[:, 0],
                z=display_z,
                colorscale=RANGE_CS,
                zmin=0,
                zmax=100,
                colorbar=_colorbar("Batt %", t),
                hovertemplate=(
                    "X: %{x:.2f}m  Y: %{y:.2f}m<br>"
                    "Return battery: %{z:.0f}%<extra></extra>"
                ),
            )
        ]
    )

    # Unreachable overlay (semi-transparent dark)
    unreachable_z = np.where(reachable, np.nan, 0.0)
    if np.any(~reachable):
        fig.add_trace(
            go.Heatmap(
                x=xg[0, :],
                y=yg[:, 0],
                z=unreachable_z,
                colorscale=[[0, "rgba(30,0,0,0.6)"], [1, "rgba(30,0,0,0.6)"]],
                showscale=False,
                hoverinfo="skip",
            )
        )

    # Lander marker
    if lander_pos is not None:
        fig.add_trace(
            go.Scatter(
                x=[lander_pos[0]],
                y=[lander_pos[1]],
                mode="markers+text",
                marker=dict(
                    symbol="diamond",
                    size=18,
                    color="#ffaa00",
                    line=dict(width=2, color="#ffffff"),
                ),
                text=["LANDER"],
                textposition="top center",
                textfont=dict(size=10, color="#ffaa00", family="Courier New"),
                name="Lander",
                showlegend=False,
                hovertemplate="LANDER<br>X: %{x:.2f}m<br>Y: %{y:.2f}m<extra></extra>",
            )
        )

    # Waypoints
    if waypoints and len(waypoints) > 0:
        wp = np.asarray(waypoints)
        labels = [f"WP{i + 1}" for i in range(len(wp))]
        fig.add_trace(
            go.Scatter(
                x=wp[:, 0],
                y=wp[:, 1],
                mode="markers+text",
                marker=dict(
                    symbol="circle",
                    size=12,
                    color=t["accent"],
                    line=dict(width=2, color="#ffffff"),
                ),
                text=labels,
                textposition="top center",
                textfont=dict(size=9, color=t["accent"], family="Courier New"),
                name="Waypoints",
                showlegend=False,
                hovertemplate="Waypoint<br>X: %{x:.2f}m<br>Y: %{y:.2f}m<extra></extra>",
            )
        )

    # Planned path through waypoints
    if wp_path is not None and len(wp_path) > 0:
        pp = np.asarray(wp_path)
        fig.add_trace(
            go.Scatter(
                x=pp[:, 0],
                y=pp[:, 1],
                mode="lines",
                line=dict(width=3, color=t["accent"], dash="solid"),
                name="Route",
                showlegend=False,
                hoverinfo="skip",
            )
        )
        if wp_cost is not None:
            fig.add_annotation(
                text=f"ROUTE: {wp_cost:.1f} Wh",
                xref="paper",
                yref="paper",
                x=0.98,
                y=0.02,
                showarrow=False,
                font=dict(size=12, color=t["accent"], family="Courier New, monospace"),
                bgcolor=t["panel_bg"],
                bordercolor=t["accent"],
                borderwidth=1,
            )

    # Battery info annotation
    fig.add_annotation(
        text=f"CHARGE: {charge_pct:.0f}% ({BATTERY_ENERGY_WH * charge_pct / 100:.0f} Wh) | "
        f"RESERVE: 25% ({BATTERY_ENERGY_WH * 0.25:.0f} Wh)",
        xref="paper",
        yref="paper",
        x=0.5,
        y=1.02,
        showarrow=False,
        font=dict(size=10, color=t["font_color"], family="Courier New, monospace"),
    )

    if lander_pos is None:
        fig.add_annotation(
            text="CLICK TO PLACE LANDER",
            xref="paper",
            yref="paper",
            x=0.5,
            y=0.5,
            showarrow=False,
            font=dict(size=16, color="#ffaa00", family="Courier New, monospace"),
        )

    fig.update_layout(**_layout_base(t), xaxis=_xaxis(t), yaxis=_yaxis(t))
    return fig


# ---------------------------------------------------------------------------
# Dash application
# ---------------------------------------------------------------------------

ROTATION_JS = """
function(n, fig) {
    if (!fig || !fig.layout) { return window.dash_clientside.no_update; }
    var angle = (n * 0.4) % 360;
    var rad   = angle * Math.PI / 180;
    var elev  = 35 * Math.PI / 180;
    var r     = 1.8;
    var newFig = JSON.parse(JSON.stringify(fig));
    newFig.layout.scene.camera = {
        eye:    {x: r*Math.cos(elev)*Math.cos(rad),
                 y: r*Math.cos(elev)*Math.sin(rad),
                 z: r*Math.sin(elev)},
        center: {x:0, y:0, z:0},
        up:     {x:0, y:0, z:1}
    };
    return newFig;
}
"""

# All layer definitions: (id_key, label)
ALL_LAYERS = [
    ("3d", "3D TERRAIN"),
    ("elevation", "ELEVATION MAP"),
    ("contour", "CONTOUR MAP"),
    ("solar", "SOLAR / SHADOW"),
    ("hazard", "HAZARD MAP"),
    ("path", "PATH PLANNER"),
    ("comms", "COMMS / LOS"),
    ("resources", "RESOURCES / ICE"),
    ("rover", "ROVER SIM"),
    ("psr", "SOLAR RADIATION"),
    ("score", "MISSION SCORE"),
    ("range", "BATTERY RANGE"),
]

# Mode descriptions — purpose and key features for each layer
LAYER_INFO = {
    "3d": (
        "3D TERRAIN VIEW",
        "Interactive 3D surface model of the lunar terrain built from LiDAR "
        "point cloud data. Rotate, pan and zoom to inspect surface features. "
        "Colour intensity shows surface reflectance.",
    ),
    "elevation": (
        "ELEVATION HEAT MAP",
        "Top-down colour-coded elevation map. Blue = low terrain, red = high "
        "terrain. Use this to identify craters, ridges and slopes at a glance.",
    ),
    "contour": (
        "TOPOGRAPHIC CONTOURS",
        "Elevation contour lines overlaid on the terrain. Closely spaced lines "
        "indicate steep slopes. Annotated contour values show height in metres.",
    ),
    "solar": (
        "SOLAR ILLUMINATION",
        "Shadow map showing which areas are sunlit vs in shadow at a given "
        "date/time. Adjust lat/lon and date above, or play the 28-day lunar "
        "cycle to see how illumination changes.",
    ),
    "hazard": (
        "TERRAIN HAZARD MAP",
        "Risk assessment combining slope steepness and surface roughness. "
        "Green = safe traversal, orange = caution, red = dangerous or "
        "impassable (slope > 15 deg).",
    ),
    "path": (
        "PATH PLANNER",
        "Click two points on the map to compute the safest route between them "
        "using shortest-path optimisation. The route avoids steep slopes and "
        "hazardous terrain. Total traversal cost is shown.",
    ),
    "comms": (
        "LINE-OF-SIGHT COMMS",
        "Radio comms coverage with RF shadow mapping. Green = clear LOS, "
        "red = blocked by terrain. Dotted contours show shadow boundaries. "
        "Use sidebar to switch between placing the base station or checking "
        "rover LOS. Moving the base recomputes the full coverage map.",
    ),
    "resources": (
        "RESOURCE OVERLAY",
        "Predicted water-ice deposit probability based on permanently shadowed "
        "regions and low elevation. Diamond markers show optimal drill sites "
        "ranked by ice concentration.",
    ),
    "rover": (
        "ROVER FOOTPRINT SIM",
        "Click any point to simulate rover placement. Shows wheel contact "
        "positions, body outline, pitch/roll angles, and ground clearance. "
        "Use the heading slider in the sidebar to rotate.",
    ),
    "psr": (
        "SOLAR RADIATION / PSR",
        "Cumulative solar radiation received over a full 28-day lunar day "
        "(kWh/m²). Accounts for sun elevation angle, surface slope, and "
        "terrain shadowing. Black = permanently shadowed regions (PSRs). "
        "Magenta contour outlines PSR boundaries.",
    ),
    "score": (
        "MISSION SUITABILITY SCORE",
        "Composite site score (0-100) on a 0.25m grid. Combines solar "
        "exposure, slope safety, rover clearance, and comms coverage. "
        "Hover to see component breakdown in the sidebar.",
    ),
    "range": (
        "BATTERY RANGE PLANNER",
        "Place a lander base station and waypoints to visualise reachable "
        "area with the rover's 24V 50Ah battery. Heatmap shows remaining "
        "battery after round-trip. Adjust charge % with the slider.",
    ),
}


def create_app(pcd_path: str):
    print(f"[PERSEUS] Loading point cloud: {pcd_path}")
    points, intensity = load_pcd(pcd_path)
    n = len(points)
    print(f"[PERSEUS] Loaded {n:,} points")

    max_3d = 50_000
    sub = max(1, n // max_3d)
    if sub > 1:
        print(f"[PERSEUS] Subsampling 3D view: 1/{sub} ({n // sub:,} points)")

    print("[PERSEUS] Interpolating terrain grid...")
    xg, yg, zg = make_terrain_grid(points, resolution=150)

    now = datetime.now(timezone.utc)
    sun0 = compute_sun_direction(now)
    illum0 = compute_shadow_map(xg, yg, zg, sun0)

    # ── Precompute all analysis layers (slower startup, faster UI) ──
    import time as _time

    _t0 = _time.monotonic()
    print("[PERSEUS] [1/8] Computing slope and hazard maps...")
    slope_deg, variance = compute_slope_map(xg, yg, zg)
    hazard = compute_hazard_map(slope_deg, variance)
    print(f"[PERSEUS]        Done ({_time.monotonic() - _t0:.1f}s)")

    _t1 = _time.monotonic()
    print(
        "[PERSEUS] [2/8] Precomputing 28-day lunar illumination cycle "
        "(28 shadow maps)..."
    )
    illum_stack, cycle_ts, solar_uptime, solar_radiation, psr_mask = (
        compute_lunar_cycle_illumination(
            xg, yg, zg, now, DEFAULT_LAT, DEFAULT_LON, n_steps=28
        )
    )
    print(f"[PERSEUS]        Done ({_time.monotonic() - _t1:.1f}s)")

    _t2 = _time.monotonic()
    print("[PERSEUS] [3/8] Generating ice deposits and drill sites...")
    ice_prob, drill_sites = generate_ice_deposits(xg, yg, zg, illum0)
    print(f"[PERSEUS]        Done ({_time.monotonic() - _t2:.1f}s)")

    # Comms: default base at terrain centre
    default_base = (
        (float(xg[0, 0]) + float(xg[0, -1])) / 2,
        (float(yg[0, 0]) + float(yg[-1, 0])) / 2,
    )
    _t3 = _time.monotonic()
    print("[PERSEUS] [4/8] Computing comms radial viewshed...")
    comms_coverage = compute_comms_coverage(xg, yg, zg, default_base)
    print(f"[PERSEUS]        Done ({_time.monotonic() - _t3:.1f}s)")

    _t4 = _time.monotonic()
    print("[PERSEUS] [5/8] Computing traversal cost grid...")
    cost_grid = compute_traversal_cost(slope_deg, hazard, solar_uptime, comms_coverage)
    print(f"[PERSEUS]        Done ({_time.monotonic() - _t4:.1f}s)")

    _t5 = _time.monotonic()
    print("[PERSEUS] [6/8] Computing mission scores (0.25m grid)...")
    score_xg, score_yg, score_grid, score_comp, score_summary = compute_mission_score(
        xg,
        yg,
        zg,
        slope_deg,
        solar_uptime,
        comms_coverage,
        hazard,
        ice_prob,
        cell_size=0.25,
    )
    print(f"[PERSEUS]        Done ({_time.monotonic() - _t5:.1f}s)")
    print("[PERSEUS] [7/8] Computing energy cost grid...")
    energy_cost_grid = compute_energy_cost_grid(slope_deg, hazard)
    print(f"[PERSEUS]        Done ({_time.monotonic() - _t5:.1f}s)")

    _t6 = _time.monotonic()
    print("[PERSEUS] [8/8] Building sparse graphs for pathfinding...")
    grid_rows, grid_cols = cost_grid.shape
    traversal_graph = build_sparse_graph(cost_grid, xg, yg)
    energy_graph = build_sparse_graph(energy_cost_grid, xg, yg)
    print(f"[PERSEUS]        Done ({_time.monotonic() - _t6:.1f}s)")

    # Cache for Dijkstra results keyed by lander grid position
    _range_cache = {}

    # Precompute shaded z-data for all cycle frames (avoids rebuilding figures)
    z_norm = (zg - zg.min()) / (zg.max() - zg.min() + 1e-9)
    shaded_stack = [z_norm * (0.3 + 0.7 * il) for il in illum_stack]

    print(f"[PERSEUS] Total precomputation: {_time.monotonic() - _t0:.1f}s")

    # Stats for header
    xr = [float(points[:, 0].min()), float(points[:, 0].max())]
    yr = [float(points[:, 1].min()), float(points[:, 1].max())]
    zr = [float(points[:, 2].min()), float(points[:, 2].max())]
    xs, ys, zs = xr[1] - xr[0], yr[1] - yr[0], zr[1] - zr[0]

    # Initial sun info
    elev0 = math.degrees(math.asin(np.clip(sun0[2], -1, 1)))
    az0 = math.degrees(math.atan2(sun0[0], sun0[1])) % 360
    status0 = "ABOVE HORIZON" if sun0[2] > 0 else "BELOW HORIZON"
    info0 = (
        f"Sun Az: {az0:.1f} | Elev: {elev0:.2f} | {status0} | "
        f"{np.mean(illum0) * 100:.0f}% illuminated"
    )

    # Build initial figures — only the default panel; others built on demand
    init_eye = dict(
        x=1.8 * math.cos(math.radians(35)),
        y=0,
        z=1.8 * math.sin(math.radians(35)),
    )

    # Cache for lazily-built figures
    _fig_cache = {}

    # Interactive panels should not be cached (they have click/slider state)
    _no_cache_keys = {"solar", "path", "comms", "rover", "range"}

    def _get_fig(key, theme="dark"):
        """Build figure for a given layer key (cached per theme)."""
        cache_key = (key, theme)
        if key not in _no_cache_keys and cache_key in _fig_cache:
            return _fig_cache[cache_key]
        builders = {
            "3d": lambda: fig_3d(points, intensity, sub, init_eye, theme),
            "elevation": lambda: fig_heatmap(xg, yg, zg, theme),
            "contour": lambda: fig_contour(xg, yg, zg, theme=theme),
            "solar": lambda: fig_shadow(xg, yg, zg, illum0, theme),
            "hazard": lambda: fig_hazard(xg, yg, hazard, slope_deg, theme),
            "path": lambda: fig_path_plan(xg, yg, hazard, theme=theme),
            "comms": lambda: fig_comms(
                xg, yg, comms_coverage, default_base, theme=theme
            ),
            "resources": lambda: fig_resources(xg, yg, ice_prob, drill_sites, theme),
            "rover": lambda: fig_rover(xg, yg, zg, slope_deg, theme=theme),
            "psr": lambda: fig_psr(xg, yg, solar_radiation, psr_mask, theme),
            "score": lambda: fig_mission_score(
                score_xg, score_yg, score_grid, score_comp, theme
            ),
            "range": lambda: fig_battery_range(
                xg,
                yg,
                np.full_like(zg, np.nan),
                np.zeros_like(zg, dtype=bool),
                None,
                theme=theme,
            ),
        }
        fig = builders[key]()
        if key not in _no_cache_keys:
            _fig_cache[cache_key] = fig
        return fig

    # Pre-build just the default panel
    _default_layer = "3d"
    _empty_fig = go.Figure()
    _empty_fig.update_layout(
        paper_bgcolor=THEMES["dark"]["panel_bg"],
        plot_bgcolor=THEMES["dark"]["panel_bg"],
        xaxis=dict(visible=False),
        yaxis=dict(visible=False),
    )

    print("[PERSEUS] Building dashboard...")

    app = Dash(__name__, suppress_callback_exceptions=True)
    app.title = "PERSEUS — Lunar Terrain Monitor"

    # --- Layout helper ---
    def _panel(label, graph_id, figure, extra_children=None):
        children = [
            html.Div(
                label,
                style={
                    "position": "absolute",
                    "top": "4px",
                    "left": "10px",
                    "fontSize": "10px",
                    "letterSpacing": "2px",
                    "zIndex": 10,
                },
                className="panel-label",
            ),
            dcc.Graph(
                id=graph_id,
                figure=figure,
                style={"height": "100%"},
                config={"displayModeBar": True, "scrollZoom": True},
            ),
        ]
        if extra_children:
            children.extend(extra_children)
        return html.Div(
            className="panel-box",
            style={
                "borderRadius": "4px",
                "position": "relative",
                "overflow": "hidden",
            },
            children=children,
        )

    # Graph IDs mapping
    graph_ids = {
        "3d": "terrain-3d",
        "elevation": "topo-heatmap",
        "contour": "contour-map",
        "solar": "shadow-map",
        "hazard": "hazard-map",
        "path": "path-plan",
        "comms": "comms-map",
        "resources": "resource-map",
        "rover": "rover-map",
        "psr": "psr-map",
        "score": "score-map",
        "range": "range-map",
    }

    # Score summary text
    sc_sum = score_summary
    score_text = (
        f"AVG: {sc_sum['mean_score']:.0f} | "
        f"BEST: {sc_sum['max_score']:.0f} | "
        f">70: {sc_sum['coverage_above_70']:.0f}%"
    )

    # --- Main layout ---
    app.layout = html.Div(
        id="main-container",
        children=[
            # Theme store
            dcc.Store(id="theme-store", data="dark"),
            # Path planning stores
            dcc.Store(id="path-start", data=None),
            dcc.Store(id="path-end", data=None),
            # Rover heading store
            dcc.Store(id="rover-heading-store", data=0),
            # Battery range stores
            dcc.Store(id="lander-pos", data=None),
            dcc.Store(id="range-waypoints", data=[]),
            dcc.Store(id="range-click-mode", data="lander"),
            # Comms base station store
            dcc.Store(id="comms-base-pos", data=list(default_base)),
            # Cycle playback store
            dcc.Store(id="cycle-frame", data=0),
            # 3D figure store for rotation
            dcc.Store(id="fig3d-store", data=_get_fig("3d").to_dict()),
            # ── Header ──
            html.Div(
                id="header-bar",
                children=[
                    html.Div(
                        style={
                            "display": "flex",
                            "justifyContent": "space-between",
                            "alignItems": "center",
                        },
                        children=[
                            html.H1(
                                "PERSEUS LUNAR TERRAIN MONITOR",
                                id="main-title",
                                style={
                                    "margin": "0",
                                    "fontSize": "22px",
                                    "letterSpacing": "4px",
                                },
                            ),
                            html.Button(
                                "LIGHT MODE",
                                id="theme-btn",
                                n_clicks=0,
                                style={
                                    "padding": "4px 12px",
                                    "cursor": "pointer",
                                    "fontFamily": "Courier New, monospace",
                                    "fontSize": "11px",
                                    "fontWeight": "bold",
                                    "borderRadius": "3px",
                                },
                            ),
                        ],
                    ),
                    html.Div(
                        id="header-stats",
                        children=[
                            html.Span(
                                "SHACKLETON CRATER",
                                style={"fontSize": "12px", "marginRight": "20px"},
                            ),
                            html.Span(
                                f"PTS: {n:,}",
                                style={"fontSize": "12px", "marginRight": "20px"},
                            ),
                            html.Span(
                                f"AREA: {xs:.1f}x{ys:.1f}m",
                                style={"fontSize": "12px", "marginRight": "20px"},
                            ),
                            html.Span(f"ELEV: {zs:.1f}m", style={"fontSize": "12px"}),
                        ],
                    ),
                ],
            ),
            # ── Sun / Cycle control bar ──
            html.Div(
                id="sun-bar",
                style={
                    "display": "flex",
                    "alignItems": "center",
                    "justifyContent": "center",
                    "gap": "10px",
                    "padding": "6px 8px",
                    "borderRadius": "4px",
                    "marginBottom": "8px",
                    "flexWrap": "wrap",
                },
                children=[
                    html.Label("SUN", style={"fontWeight": "bold", "fontSize": "12px"}),
                    html.Label("Lat:", style={"fontSize": "11px"}),
                    dcc.Input(
                        id="moon-lat",
                        type="number",
                        value=DEFAULT_LAT,
                        step=0.1,
                        style={
                            "width": "70px",
                            "fontSize": "11px",
                            "fontFamily": "Courier New, monospace",
                            "padding": "3px 5px",
                        },
                    ),
                    html.Label("Lon:", style={"fontSize": "11px"}),
                    dcc.Input(
                        id="moon-lon",
                        type="number",
                        value=DEFAULT_LON,
                        step=0.1,
                        style={
                            "width": "70px",
                            "fontSize": "11px",
                            "fontFamily": "Courier New, monospace",
                            "padding": "3px 5px",
                        },
                    ),
                    html.Label("Date:", style={"fontSize": "11px"}),
                    dcc.Input(
                        id="sun-date",
                        type="date",
                        value=now.strftime("%Y-%m-%d"),
                        style={
                            "fontSize": "11px",
                            "fontFamily": "Courier New, monospace",
                            "padding": "3px 5px",
                        },
                    ),
                    html.Label("Hour:", style={"fontSize": "11px"}),
                    dcc.Slider(
                        id="sun-hour",
                        min=0,
                        max=23,
                        step=1,
                        value=now.hour,
                        marks={
                            h: {"label": f"{h:02d}", "style": {"fontSize": "9px"}}
                            for h in range(0, 24, 6)
                        },
                        tooltip={"placement": "bottom", "always_visible": False},
                    ),
                    html.Button(
                        "UPDATE",
                        id="sun-btn",
                        style={
                            "padding": "4px 12px",
                            "cursor": "pointer",
                            "fontFamily": "Courier New, monospace",
                            "fontSize": "11px",
                            "fontWeight": "bold",
                        },
                    ),
                    html.Span("|", style={"fontSize": "14px"}),
                    html.Button(
                        "PLAY CYCLE",
                        id="cycle-play-btn",
                        n_clicks=0,
                        style={
                            "padding": "4px 12px",
                            "cursor": "pointer",
                            "fontFamily": "Courier New, monospace",
                            "fontSize": "11px",
                            "fontWeight": "bold",
                        },
                    ),
                    html.Span(
                        id="sun-info",
                        children=info0,
                        style={"fontSize": "10px", "minWidth": "180px"},
                    ),
                    dcc.Interval(
                        id="cycle-tick", interval=400, disabled=True, n_intervals=0
                    ),
                ],
            ),
            # ── Body: sidebar + panels ──
            html.Div(
                style={
                    "display": "flex",
                    "height": "calc(100vh - 160px)",
                    "gap": "8px",
                },
                children=[
                    # Sidebar
                    html.Div(
                        id="sidebar",
                        style={
                            "width": "200px",
                            "minWidth": "200px",
                            "borderRadius": "4px",
                            "padding": "8px",
                            "overflowY": "auto",
                        },
                        children=[
                            html.Div(
                                "LAYERS",
                                style={
                                    "fontWeight": "bold",
                                    "fontSize": "12px",
                                    "letterSpacing": "2px",
                                    "marginBottom": "6px",
                                },
                            ),
                            dcc.RadioItems(
                                id="layer-toggles",
                                options=[
                                    {"label": f" {lbl}", "value": key}
                                    for key, lbl in ALL_LAYERS
                                ],
                                value=_default_layer,
                                style={"fontSize": "11px"},
                                inputStyle={"marginRight": "4px"},
                                labelStyle={
                                    "display": "block",
                                    "padding": "3px 0",
                                    "cursor": "pointer",
                                },
                            ),
                            html.Hr(style={"margin": "8px 0"}),
                            html.Div(
                                "ROVER",
                                style={
                                    "fontWeight": "bold",
                                    "fontSize": "11px",
                                    "letterSpacing": "2px",
                                    "marginBottom": "4px",
                                },
                            ),
                            html.Label("Heading:", style={"fontSize": "10px"}),
                            dcc.Slider(
                                id="rover-heading",
                                min=0,
                                max=350,
                                step=10,
                                value=0,
                                marks={
                                    d: {"label": f"{d}", "style": {"fontSize": "8px"}}
                                    for d in range(0, 360, 90)
                                },
                                tooltip={"placement": "bottom"},
                            ),
                            html.Hr(style={"margin": "8px 0"}),
                            html.Div(
                                "COMMS / LOS",
                                style={
                                    "fontWeight": "bold",
                                    "fontSize": "11px",
                                    "letterSpacing": "2px",
                                    "marginBottom": "4px",
                                },
                            ),
                            html.Label("Click mode:", style={"fontSize": "10px"}),
                            dcc.RadioItems(
                                id="comms-mode",
                                options=[
                                    {"label": " Move Base Station", "value": "base"},
                                    {"label": " Check Rover LOS", "value": "rover"},
                                ],
                                value="base",
                                style={"fontSize": "10px", "marginBottom": "4px"},
                                inputStyle={"marginRight": "3px"},
                                labelStyle={"display": "block", "padding": "1px 0"},
                            ),
                            html.Div(
                                id="comms-info",
                                style={
                                    "fontSize": "9px",
                                    "lineHeight": "1.5",
                                    "marginTop": "4px",
                                    "padding": "4px",
                                    "border": "1px solid var(--grid-clr)",
                                    "borderRadius": "3px",
                                    "backgroundColor": "var(--page-bg)",
                                },
                                children=[
                                    html.Div(
                                        id="comms-base-info",
                                        children=f"Base: ({default_base[0]:.1f}, "
                                        f"{default_base[1]:.1f})",
                                    ),
                                    html.Div(
                                        id="comms-rover-info",
                                        children="Rover: click map",
                                    ),
                                ],
                            ),
                            html.Hr(style={"margin": "8px 0"}),
                            html.Div(
                                "BATTERY RANGE",
                                style={
                                    "fontWeight": "bold",
                                    "fontSize": "11px",
                                    "letterSpacing": "2px",
                                    "marginBottom": "4px",
                                },
                            ),
                            html.Label("Click mode:", style={"fontSize": "10px"}),
                            dcc.RadioItems(
                                id="range-mode",
                                options=[
                                    {"label": " Place Lander", "value": "lander"},
                                    {"label": " Add Waypoint", "value": "waypoint"},
                                ],
                                value="lander",
                                style={"fontSize": "10px", "marginBottom": "4px"},
                                inputStyle={"marginRight": "3px"},
                                labelStyle={"display": "block", "padding": "1px 0"},
                            ),
                            html.Label("Battery charge %:", style={"fontSize": "10px"}),
                            dcc.Slider(
                                id="battery-charge",
                                min=10,
                                max=100,
                                step=5,
                                value=100,
                                marks={
                                    v: {"label": f"{v}%", "style": {"fontSize": "8px"}}
                                    for v in (25, 50, 75, 100)
                                },
                                tooltip={"placement": "bottom"},
                            ),
                            html.Div(
                                id="range-info",
                                style={
                                    "fontSize": "9px",
                                    "lineHeight": "1.5",
                                    "marginTop": "4px",
                                    "padding": "4px",
                                    "border": "1px solid var(--grid-clr)",
                                    "borderRadius": "3px",
                                    "backgroundColor": "var(--page-bg)",
                                },
                                children=[
                                    html.Div(
                                        "24V 50Ah (1200Wh)",
                                        style={"color": "var(--accent)"},
                                    ),
                                    html.Div("Idle: 0.5A | Flat: 4A | Max: 40A"),
                                    html.Div("Reserve: 25% (300Wh)"),
                                    html.Div(
                                        id="range-lander-info",
                                        children="Lander: not placed",
                                    ),
                                    html.Div(
                                        id="range-wp-info", children="Waypoints: 0"
                                    ),
                                ],
                            ),
                            html.Button(
                                "CLEAR WAYPOINTS",
                                id="range-clear-btn",
                                n_clicks=0,
                                style={
                                    "width": "100%",
                                    "padding": "3px",
                                    "marginTop": "4px",
                                    "cursor": "pointer",
                                    "fontFamily": "Courier New, monospace",
                                    "fontSize": "10px",
                                    "backgroundColor": "var(--btn-bg)",
                                    "color": "var(--font-clr)",
                                    "border": "1px solid var(--font-clr)",
                                },
                            ),
                            html.Hr(style={"margin": "8px 0"}),
                            html.Div(
                                "MISSION SCORE",
                                style={
                                    "fontWeight": "bold",
                                    "fontSize": "11px",
                                    "letterSpacing": "2px",
                                    "marginBottom": "6px",
                                },
                            ),
                            # Live readout panel — updates on hover over score map
                            html.Div(
                                id="score-live-panel",
                                style={
                                    "fontSize": "10px",
                                    "lineHeight": "1.7",
                                    "padding": "6px",
                                    "borderRadius": "3px",
                                    "border": "1px solid var(--grid-clr)",
                                    "backgroundColor": "var(--page-bg)",
                                },
                                children=[
                                    html.Div(
                                        id="score-live-title",
                                        children="HOVER OVER MAP",
                                        style={
                                            "fontWeight": "bold",
                                            "marginBottom": "4px",
                                            "color": "var(--accent)",
                                        },
                                    ),
                                    html.Div(
                                        [
                                            html.Div(
                                                id="score-live-coords",
                                                children="X: --  Y: --",
                                            ),
                                            html.Div(
                                                id="score-live-latlon",
                                                children=f"Lat: {DEFAULT_LAT:.4f}  "
                                                f"Lon: {DEFAULT_LON:.4f}",
                                            ),
                                        ],
                                        style={"marginBottom": "4px"},
                                    ),
                                    html.Table(
                                        style={"width": "100%", "fontSize": "10px"},
                                        children=[
                                            html.Tr(
                                                [
                                                    html.Td("Solar:"),
                                                    html.Td(
                                                        id="score-live-solar",
                                                        children="--/25",
                                                        style={"textAlign": "right"},
                                                    ),
                                                ]
                                            ),
                                            html.Tr(
                                                [
                                                    html.Td("Slope:"),
                                                    html.Td(
                                                        id="score-live-slope",
                                                        children="--/25",
                                                        style={"textAlign": "right"},
                                                    ),
                                                ]
                                            ),
                                            html.Tr(
                                                [
                                                    html.Td("Clearance:"),
                                                    html.Td(
                                                        id="score-live-clear",
                                                        children="--/25",
                                                        style={"textAlign": "right"},
                                                    ),
                                                ]
                                            ),
                                            html.Tr(
                                                [
                                                    html.Td("Comms:"),
                                                    html.Td(
                                                        id="score-live-comms",
                                                        children="--/25",
                                                        style={"textAlign": "right"},
                                                    ),
                                                ]
                                            ),
                                            html.Tr(
                                                style={
                                                    "fontWeight": "bold",
                                                    "borderTop": "1px solid var(--grid-clr)",
                                                },
                                                children=[
                                                    html.Td("TOTAL:"),
                                                    html.Td(
                                                        id="score-live-total",
                                                        children="--/100",
                                                        style={"textAlign": "right"},
                                                    ),
                                                ],
                                            ),
                                        ],
                                    ),
                                ],
                            ),
                            html.Div(
                                style={"marginTop": "6px", "fontSize": "10px"},
                                children=[
                                    html.Div(id="score-summary", children=score_text),
                                    html.Div(
                                        f"Best: ({sc_sum['best_location'][0]:.1f}, "
                                        f"{sc_sum['best_location'][1]:.1f})"
                                    ),
                                ],
                            ),
                        ],
                    ),
                    # Panel column: mode description + map
                    html.Div(
                        style={
                            "flex": "1",
                            "display": "flex",
                            "flexDirection": "column",
                            "overflow": "hidden",
                        },
                        children=[
                            # Mode description box
                            html.Div(
                                id="mode-desc-box",
                                style={
                                    "padding": "10px 14px",
                                    "marginBottom": "6px",
                                    "borderRadius": "4px",
                                    "border": "1px solid var(--accent)",
                                    "backgroundColor": "var(--panel-bg)",
                                    "minHeight": "50px",
                                },
                                children=[
                                    html.Div(
                                        id="mode-desc-title",
                                        children=LAYER_INFO[_default_layer][0],
                                        style={
                                            "fontSize": "24px",
                                            "fontWeight": "bold",
                                            "letterSpacing": "3px",
                                            "color": "var(--accent)",
                                            "marginBottom": "4px",
                                        },
                                    ),
                                    html.Div(
                                        id="mode-desc-text",
                                        children=LAYER_INFO[_default_layer][1],
                                        style={
                                            "fontSize": "16px",
                                            "lineHeight": "1.4",
                                            "color": "var(--font-clr)",
                                        },
                                    ),
                                ],
                            ),
                            # Single full-size panel
                            html.Div(
                                id="panel-area",
                                style={
                                    "flex": "1",
                                    "position": "relative",
                                },
                                children=[
                                    html.Div(
                                        id=f"panel-wrap-{key}",
                                        children=[
                                            _panel(
                                                lbl,
                                                graph_ids[key],
                                                _get_fig(key)
                                                if key == _default_layer
                                                else _empty_fig,
                                                extra_children=(
                                                    [
                                                        dcc.Interval(
                                                            id="rot-tick",
                                                            interval=100,
                                                            n_intervals=0,
                                                        )
                                                    ]
                                                    if key == "3d"
                                                    else None
                                                ),
                                            )
                                        ],
                                        style={
                                            "display": "block"
                                            if key == _default_layer
                                            else "none",
                                            "height": "100%",
                                        },
                                    )
                                    for key, lbl in ALL_LAYERS
                                ],
                            ),
                        ],
                    ),
                ],
            ),
        ],
    )

    # ── CSS injection for theming ──
    app.index_string = """
    <!DOCTYPE html>
    <html>
    <head>
        {%metas%}{%favicon%}{%css%}
        <title>{%title%}</title>
        <style>
        .theme-dark {
            --page-bg: #0a0a12; --panel-bg: #0d1117; --grid-clr: #1a2332;
            --font-clr: #00ff88; --accent: #00ccff; --muted: #aaa;
            --input-bg: #1a1a2e; --btn-bg: #1a3322;
        }
        .theme-light {
            --page-bg: #f0f2f5; --panel-bg: #ffffff; --grid-clr: #cccccc;
            --font-clr: #003322; --accent: #0066cc; --muted: #666;
            --input-bg: #ffffff; --btn-bg: #d4edda;
        }
        #main-container {
            background-color: var(--page-bg); min-height: 100vh; padding: 8px;
            font-family: 'Courier New', monospace; color: var(--font-clr);
            transition: background-color 0.3s, color 0.3s;
        }
        #header-bar {
            text-align: center; border-bottom: 1px solid var(--accent);
            padding-bottom: 6px; margin-bottom: 8px;
        }
        #main-title { color: var(--accent); }
        #theme-btn {
            background-color: var(--btn-bg); color: var(--font-clr);
            border: 1px solid var(--font-clr);
        }
        #sun-bar {
            background-color: var(--panel-bg); border: 1px solid var(--grid-clr);
        }
        #sun-bar input, #sun-bar .rc-slider {
            background-color: var(--input-bg); color: var(--font-clr);
            border: 1px solid var(--grid-clr);
        }
        #sun-btn, #cycle-play-btn {
            background-color: var(--btn-bg); color: var(--font-clr);
            border: 1px solid var(--font-clr);
        }
        #sidebar {
            background-color: var(--panel-bg); border: 1px solid var(--grid-clr);
        }
        .panel-box { border: 1px solid var(--grid-clr); height: 100%; }
        #panel-area { min-height: 0; }
        .panel-label { color: var(--accent); }
        #sun-info { color: var(--muted); }
        #score-summary { color: var(--accent); }
        </style>
    </head>
    <body>
        {%app_entry%}
        <footer>{%config%}{%scripts%}{%renderer%}</footer>
    </body>
    </html>
    """

    # ── Callbacks ──

    # 1. Theme toggle
    @app.callback(
        Output("main-container", "className"),
        Output("theme-btn", "children"),
        Output("theme-store", "data"),
        Input("theme-btn", "n_clicks"),
        State("theme-store", "data"),
    )
    def toggle_theme(n, current):
        if not n:
            return "theme-dark", "LIGHT MODE", "dark"
        new = "light" if current == "dark" else "dark"
        btn_text = "DARK MODE" if new == "light" else "LIGHT MODE"
        return f"theme-{new}", btn_text, new

    # 2. Layer toggle — show one panel at a time, build figure on demand
    _layer_style_outputs = [
        Output(f"panel-wrap-{key}", "style") for key, _ in ALL_LAYERS
    ]
    _layer_fig_outputs = [
        Output(graph_ids[key], "figure", allow_duplicate=True) for key, _ in ALL_LAYERS
    ]

    @app.callback(
        _layer_style_outputs
        + _layer_fig_outputs
        + [Output("mode-desc-title", "children"), Output("mode-desc-text", "children")],
        Input("layer-toggles", "value"),
        State("theme-store", "data"),
        prevent_initial_call=True,
    )
    def toggle_layers(selected, theme):
        styles = []
        figs = []
        for key, _ in ALL_LAYERS:
            if key == selected:
                styles.append({"display": "block", "height": "100%"})
                figs.append(_get_fig(key, theme))
            else:
                styles.append({"display": "none"})
                figs.append(no_update)
        title, desc = LAYER_INFO.get(selected, ("", ""))
        return styles + figs + [title, desc]

    # 3. Client-side 3D rotation
    app.clientside_callback(
        ROTATION_JS,
        Output("terrain-3d", "figure"),
        Input("rot-tick", "n_intervals"),
        State("fig3d-store", "data"),
    )

    # 4. Shadow / sun update
    @app.callback(
        Output("shadow-map", "figure"),
        Output("sun-info", "children"),
        Input("sun-btn", "n_clicks"),
        State("sun-date", "value"),
        State("sun-hour", "value"),
        State("moon-lat", "value"),
        State("moon-lon", "value"),
        State("theme-store", "data"),
        prevent_initial_call=True,
    )
    def update_shadow(_, date_str, hour, lat, lon, theme):
        try:
            dt = datetime.strptime(date_str, "%Y-%m-%d").replace(
                hour=int(hour), tzinfo=timezone.utc
            )
        except Exception:
            dt = now

        lat_v = max(-90.0, min(90.0, float(lat) if lat is not None else DEFAULT_LAT))
        lon_v = max(-180.0, min(180.0, float(lon) if lon is not None else DEFAULT_LON))

        sd = compute_sun_direction(dt, lat_v, lon_v)
        il = compute_shadow_map(xg, yg, zg, sd)
        pct = float(np.mean(il) * 100)

        el = math.degrees(math.asin(np.clip(sd[2], -1, 1)))
        az = math.degrees(math.atan2(sd[0], sd[1])) % 360
        st = "ABOVE HORIZON" if sd[2] > 0 else "BELOW HORIZON"
        info = (
            f"Pos: {lat_v:.1f}N {lon_v:.1f}E | "
            f"Sun Az: {az:.1f} | Elev: {el:.2f} | {st} | {pct:.0f}% lit"
        )

        return fig_shadow(xg, yg, zg, il, theme), info

    # 5. Cycle playback toggle
    @app.callback(
        Output("cycle-tick", "disabled"),
        Output("cycle-play-btn", "children"),
        Input("cycle-play-btn", "n_clicks"),
        State("cycle-tick", "disabled"),
    )
    def toggle_cycle(n, currently_disabled):
        if not n:
            raise PreventUpdate
        playing = currently_disabled  # if was disabled, now we play
        return not playing, "STOP CYCLE" if playing else "PLAY CYCLE"

    # 6. Cycle playback tick
    @app.callback(
        Output("shadow-map", "figure", allow_duplicate=True),
        Output("sun-info", "children", allow_duplicate=True),
        Output("cycle-frame", "data"),
        Input("cycle-tick", "n_intervals"),
        State("cycle-frame", "data"),
        State("theme-store", "data"),
        prevent_initial_call=True,
    )
    def cycle_tick(_, frame, theme):
        frame = (frame + 1) % len(illum_stack)
        ts = cycle_ts[frame]
        pct = float(np.mean(illum_stack[frame]) * 100)
        info = (
            f"CYCLE DAY {frame * 28.0 / len(illum_stack):.1f}/28 | "
            f"{ts.strftime('%Y-%m-%d %H:%M')} UTC | {pct:.0f}% lit"
        )
        # Use precomputed shaded data — only rebuild figure with cached z
        t = _t(theme)
        fig = go.Figure(
            data=[
                go.Heatmap(
                    x=xg[0, :],
                    y=yg[:, 0],
                    z=shaded_stack[frame],
                    colorscale=SHADOW_CS,
                    showscale=False,
                    hovertemplate="X: %{x:.2f}m<br>Y: %{y:.2f}m<extra></extra>",
                )
            ]
        )
        fig.update_layout(**_layout_base(t), xaxis=_xaxis(t), yaxis=_yaxis(t))
        return fig, info, frame

    # 7. Path planning — click to set start/end
    @app.callback(
        Output("path-plan", "figure"),
        Output("path-start", "data"),
        Output("path-end", "data"),
        Input("path-plan", "clickData"),
        State("path-start", "data"),
        State("path-end", "data"),
        State("theme-store", "data"),
        prevent_initial_call=True,
    )
    def path_click(click, start, end, theme):
        if not click or "points" not in click:
            raise PreventUpdate
        pt = click["points"][0]
        px, py = pt.get("x"), pt.get("y")
        if px is None or py is None:
            raise PreventUpdate

        if start is None:
            # Set start
            return (
                fig_path_plan(xg, yg, hazard, start_xy=(px, py), theme=theme),
                [px, py],
                None,
            )
        elif end is None:
            # Set end and compute path
            start_xy = tuple(start)
            end_xy = (px, py)
            path_coords, total_cost = find_path(
                traversal_graph, cost_grid, xg, yg, start_xy, end_xy
            )
            if path_coords is None:
                # No path found — show annotation
                f = fig_path_plan(
                    xg, yg, hazard, start_xy=start_xy, end_xy=end_xy, theme=theme
                )
                f.add_annotation(
                    text="NO SAFE PATH FOUND",
                    xref="paper",
                    yref="paper",
                    x=0.5,
                    y=0.5,
                    showarrow=False,
                    font=dict(
                        size=18, color="#cc0000", family="Courier New, monospace"
                    ),
                )
                return f, list(start), [px, py]
            return (
                fig_path_plan(
                    xg, yg, hazard, path_coords, start_xy, end_xy, total_cost, theme
                ),
                list(start),
                [px, py],
            )
        else:
            # Reset — new start
            return (
                fig_path_plan(xg, yg, hazard, start_xy=(px, py), theme=theme),
                [px, py],
                None,
            )

    # 8. Comms LOS — click to place base or rover
    @app.callback(
        Output("comms-map", "figure"),
        Output("comms-base-pos", "data"),
        Output("comms-base-info", "children"),
        Output("comms-rover-info", "children"),
        Input("comms-map", "clickData"),
        State("comms-mode", "value"),
        State("comms-base-pos", "data"),
        State("theme-store", "data"),
        prevent_initial_call=True,
    )
    def comms_click(click, mode, base_stored, theme):
        if not click or "points" not in click:
            raise PreventUpdate
        pt = click["points"][0]
        px, py = pt.get("x"), pt.get("y")
        if px is None or py is None:
            raise PreventUpdate

        base_pos = tuple(base_stored) if base_stored else default_base

        if mode == "base":
            # Relocate base station and recompute coverage
            base_pos = (px, py)
            new_coverage = compute_comms_coverage(xg, yg, zg, base_pos)
            fig = fig_comms(xg, yg, new_coverage, base_pos, theme=theme)
            return (
                fig,
                list(base_pos),
                f"Base: ({px:.1f}, {py:.1f})",
                "Rover: click map",
            )
        else:
            # Place rover and show LOS trace
            rover_pos = (px, py)
            cur_coverage = compute_comms_coverage(xg, yg, zg, base_pos)
            los_data = compute_line_of_sight(xg, yg, zg, base_pos, rover_pos)
            fig = fig_comms(xg, yg, cur_coverage, base_pos, rover_pos, los_data, theme)
            status = "CLEAR" if los_data["visible"] else "BLOCKED"
            return (
                fig,
                list(base_pos),
                f"Base: ({base_pos[0]:.1f}, {base_pos[1]:.1f})",
                f"Rover: ({px:.1f}, {py:.1f}) [{status}]",
            )

    # 9. Rover sim — click to place, slider for heading
    @app.callback(
        Output("rover-map", "figure"),
        Input("rover-map", "clickData"),
        Input("rover-heading", "value"),
        State("theme-store", "data"),
        prevent_initial_call=True,
    )
    def rover_click(click, heading, theme):
        if not click or "points" not in click:
            if heading is not None:
                # Heading changed but no click yet
                raise PreventUpdate
            raise PreventUpdate
        pt = click["points"][0]
        px, py = pt.get("x"), pt.get("y")
        if px is None or py is None:
            raise PreventUpdate

        heading = heading or 0
        rover_data = compute_rover_footprint(xg, yg, zg, (px, py), heading)
        return fig_rover(xg, yg, zg, slope_deg, rover_data, theme)

    # 10. Battery range — click to place lander / add waypoints
    @app.callback(
        Output("range-map", "figure"),
        Output("lander-pos", "data"),
        Output("range-waypoints", "data"),
        Output("range-lander-info", "children"),
        Output("range-wp-info", "children"),
        Input("range-map", "clickData"),
        Input("battery-charge", "value"),
        Input("range-clear-btn", "n_clicks"),
        State("range-mode", "value"),
        State("lander-pos", "data"),
        State("range-waypoints", "data"),
        State("theme-store", "data"),
        prevent_initial_call=True,
    )
    def range_click(click, charge_pct, clear_n, mode, lander, waypoints, theme):
        triggered = ctx.triggered_id
        charge_pct = charge_pct or 100

        def _cached_range(lp, pct):
            """Run Dijkstra with caching by lander grid position."""
            grid_pos = _world_to_grid(lp, xg, yg)
            cache_key = (grid_pos, pct)
            if cache_key in _range_cache:
                return _range_cache[cache_key]
            result = compute_battery_range(
                energy_graph, grid_rows, grid_cols, xg, yg, lp, pct
            )
            _range_cache[cache_key] = result
            return result

        # Handle clear button
        if triggered == "range-clear-btn":
            waypoints = []
            if lander is not None:
                lp = tuple(lander)
                _, reachable, range_pct = _cached_range(lp, charge_pct)
                fig = fig_battery_range(
                    xg, yg, range_pct, reachable, lp, charge_pct=charge_pct, theme=theme
                )
                return (
                    fig,
                    lander,
                    [],
                    f"Lander: ({lp[0]:.1f}, {lp[1]:.1f})",
                    "Waypoints: 0",
                )
            return (
                fig_battery_range(
                    xg,
                    yg,
                    np.full_like(zg, np.nan),
                    np.zeros_like(zg, dtype=bool),
                    None,
                    charge_pct=charge_pct,
                    theme=theme,
                ),
                None,
                [],
                "Lander: not placed",
                "Waypoints: 0",
            )

        # Handle battery slider change (no click)
        if triggered == "battery-charge":
            if lander is not None:
                lp = tuple(lander)
                _, reachable, range_pct = _cached_range(lp, charge_pct)
                wp_path = None
                wp_cost = None
                if waypoints and len(waypoints) > 0:
                    wp_path, wp_cost = _compute_wp_route(
                        lp, waypoints, traversal_graph, cost_grid, xg, yg
                    )
                fig = fig_battery_range(
                    xg,
                    yg,
                    range_pct,
                    reachable,
                    lp,
                    waypoints,
                    wp_path,
                    wp_cost,
                    charge_pct,
                    theme,
                )
                return (
                    fig,
                    lander,
                    waypoints or [],
                    f"Lander: ({lp[0]:.1f}, {lp[1]:.1f})",
                    f"Waypoints: {len(waypoints or [])}",
                )
            raise PreventUpdate

        # Handle map click
        if not click or "points" not in click:
            raise PreventUpdate
        pt = click["points"][0]
        px, py = pt.get("x"), pt.get("y")
        if px is None or py is None:
            raise PreventUpdate

        if waypoints is None:
            waypoints = []

        if mode == "lander":
            lander = [px, py]
            lp = (px, py)
            _range_cache.clear()  # new lander = invalidate cache
            _, reachable, range_pct = _cached_range(lp, charge_pct)
            wp_path = None
            wp_cost = None
            if waypoints and len(waypoints) > 0:
                wp_path, wp_cost = _compute_wp_route(
                    lp, waypoints, traversal_graph, cost_grid, xg, yg
                )
            fig = fig_battery_range(
                xg,
                yg,
                range_pct,
                reachable,
                lp,
                waypoints,
                wp_path,
                wp_cost,
                charge_pct,
                theme,
            )
            return (
                fig,
                lander,
                waypoints,
                f"Lander: ({px:.1f}, {py:.1f})",
                f"Waypoints: {len(waypoints)}",
            )
        else:
            # Add waypoint
            waypoints = waypoints + [[px, py]]
            if lander is not None:
                lp = tuple(lander)
                _, reachable, range_pct = _cached_range(lp, charge_pct)
                wp_path, wp_cost = _compute_wp_route(
                    lp, waypoints, traversal_graph, cost_grid, xg, yg
                )
                fig = fig_battery_range(
                    xg,
                    yg,
                    range_pct,
                    reachable,
                    lp,
                    waypoints,
                    wp_path,
                    wp_cost,
                    charge_pct,
                    theme,
                )
                return (
                    fig,
                    lander,
                    waypoints,
                    f"Lander: ({lp[0]:.1f}, {lp[1]:.1f})",
                    f"Waypoints: {len(waypoints)}",
                )
            else:
                fig = fig_battery_range(
                    xg,
                    yg,
                    np.full_like(zg, np.nan),
                    np.zeros_like(zg, dtype=bool),
                    None,
                    waypoints,
                    charge_pct=charge_pct,
                    theme=theme,
                )
                return (
                    fig,
                    None,
                    waypoints,
                    "Lander: not placed",
                    f"Waypoints: {len(waypoints)}",
                )

    def _compute_wp_route(lander_pos, waypoints, graph, cost_g, xg_, yg_):
        """Compute route: lander -> WP1 -> WP2 -> ... -> lander."""
        all_points = [tuple(lander_pos)]
        for wp in waypoints:
            all_points.append(tuple(wp))
        all_points.append(tuple(lander_pos))  # return to lander

        full_path = []
        total_cost = 0.0
        for i in range(len(all_points) - 1):
            seg_path, seg_cost = find_path(
                graph, cost_g, xg_, yg_, all_points[i], all_points[i + 1]
            )
            if seg_path is None:
                return None, None
            if full_path:
                seg_path = seg_path[1:]  # avoid duplicating junction point
            full_path.extend(seg_path)
            total_cost += seg_cost
        return full_path, total_cost

    # 11. Mission score live readout — update sidebar on hover
    @app.callback(
        Output("score-live-title", "children"),
        Output("score-live-coords", "children"),
        Output("score-live-latlon", "children"),
        Output("score-live-solar", "children"),
        Output("score-live-slope", "children"),
        Output("score-live-clear", "children"),
        Output("score-live-comms", "children"),
        Output("score-live-total", "children"),
        Output("score-live-total", "style"),
        Input("score-map", "hoverData"),
        State("moon-lat", "value"),
        State("moon-lon", "value"),
        prevent_initial_call=True,
    )
    def score_hover(hover, site_lat, site_lon):
        if not hover or "points" not in hover:
            raise PreventUpdate
        pt = hover["points"][0]
        px, py = pt.get("x"), pt.get("y")
        cd = pt.get("customdata")
        if px is None or py is None or cd is None:
            raise PreventUpdate

        solar_v, slope_v, clear_v, comms_v, total_v = cd

        # Convert local X/Y offset to approximate lat/lon
        # At the Moon's south pole, 1 deg latitude ~ 30.3 km
        # Longitude scaling depends on latitude
        lat_base = float(site_lat) if site_lat is not None else DEFAULT_LAT
        lon_base = float(site_lon) if site_lon is not None else DEFAULT_LON
        m_per_deg_lat = 30300.0  # ~30.3 km per degree on the Moon
        cos_lat = max(abs(math.cos(math.radians(lat_base))), 0.001)
        m_per_deg_lon = 30300.0 * cos_lat

        approx_lat = lat_base + py / m_per_deg_lat
        approx_lon = lon_base + px / m_per_deg_lon

        # Color total based on score
        if total_v >= 75:
            total_color = "#00ff88"
        elif total_v >= 50:
            total_color = "#ccaa00"
        else:
            total_color = "#cc0000"

        return (
            f"SITE SCORE: {total_v:.0f}/100",
            f"X: {px:.2f}m  Y: {py:.2f}m",
            f"Lat: {approx_lat:.6f}  Lon: {approx_lon:.6f}",
            f"{solar_v:.1f}/25",
            f"{slope_v:.1f}/25",
            f"{clear_v:.1f}/25",
            f"{comms_v:.1f}/25",
            f"{total_v:.0f}/100",
            {"textAlign": "right", "color": total_color},
        )

    # 11. Theme change — clear cache, rebuild active panel
    _theme_all_outputs = [
        Output(graph_ids[key], "figure", allow_duplicate=True) for key, _ in ALL_LAYERS
    ] + [Output("fig3d-store", "data", allow_duplicate=True)]

    @app.callback(
        _theme_all_outputs,
        Input("theme-store", "data"),
        State("layer-toggles", "value"),
        prevent_initial_call=True,
    )
    def rebuild_on_theme(theme, active_layer):
        _fig_cache.clear()
        results = []
        for key, _ in ALL_LAYERS:
            if key == active_layer:
                results.append(_get_fig(key, theme))
            else:
                results.append(no_update)
        # 3D store
        results.append(fig_3d(points, intensity, sub, init_eye, theme).to_dict())
        return results

    return app


# ---------------------------------------------------------------------------
# CLI entry point
# ---------------------------------------------------------------------------


def main():
    parser = argparse.ArgumentParser(description="Perseus Lunar PCD Viewer")
    parser.add_argument("pcd_file", help="Path to a .pcd point cloud file")
    parser.add_argument("--port", type=int, default=8050)
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--debug", action="store_true")
    args = parser.parse_args()

    app = create_app(args.pcd_file)

    print(f"\n{'=' * 60}")
    print("  PERSEUS LUNAR TERRAIN MONITOR")
    print("  Shackleton Crater — South Pole")
    print(f"  Dashboard: http://localhost:{args.port}")
    print(f"{'=' * 60}\n")

    app.run(host=args.host, port=args.port, debug=args.debug)


if __name__ == "__main__":
    main()
