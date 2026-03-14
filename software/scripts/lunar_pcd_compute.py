#!/usr/bin/env python3
"""
Perseus Lunar PCD — Computation Engine

Pure computation functions, colour scales, themes, and constants extracted
from lunar_pcd_viewer.py.  Shared by the Dash browser app and the PyQt5
native desktop viewer.
"""

import math
from datetime import datetime, timedelta, timezone

import numpy as np
from scipy.interpolate import griddata
from scipy.ndimage import gaussian_filter, maximum_filter, uniform_filter, zoom
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import dijkstra as sp_dijkstra


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
    n_rays = 360
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
    """Per-cell traversal cost.

    Impassable if slope > 15 deg OR in a comms dead zone (coverage < 0.1).
    Cells with weak signal (< 0.3) receive a heavy penalty so the planner
    strongly prefers routes that maintain radio contact with the lander.
    """
    cost = np.full_like(slope_deg, np.inf, dtype=np.float64)
    passable = (slope_deg <= 15.0) & (comms_coverage >= 0.1)
    # Heavy penalty for weak comms: 1/coverage blows up near zero
    comms_penalty = np.where(
        comms_coverage[passable] > 0.1,
        (1.0 / (comms_coverage[passable] + 0.05)) * 2.0,
        50.0,
    )
    cost[passable] = np.clip(
        np.exp(slope_deg[passable] / 10.0)
        + hazard[passable] * 5.0
        - solar_uptime[passable] * 2.0
        + comms_penalty,
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
    for node_id in path_nodes:
        r, c = divmod(node_id, cols)
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


def compute_wp_route(lander_pos, waypoints, graph, cost_grid, xg, yg):
    """Compute route: lander -> WP1 -> WP2 -> ... -> lander."""
    all_points = [tuple(lander_pos)]
    for wp in waypoints:
        all_points.append(tuple(wp))
    all_points.append(tuple(lander_pos))  # return to lander

    full_path = []
    total_cost = 0.0
    for i in range(len(all_points) - 1):
        seg_path, seg_cost = find_path(
            graph, cost_grid, xg, yg, all_points[i], all_points[i + 1]
        )
        if seg_path is None:
            return None, None
        if full_path:
            seg_path = seg_path[1:]  # avoid duplicating junction point
        full_path.extend(seg_path)
        total_cost += seg_cost
    return full_path, total_cost


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


# ---------------------------------------------------------------------------
# Colour scale → pyqtgraph LUT conversion helper
# ---------------------------------------------------------------------------


def colorscale_to_lut(cs, n_pts=256):
    """Convert a Plotly-style colorscale to a (n_pts, 4) uint8 RGBA LUT.

    Each element of *cs* is ``[position, "#rrggbb"]`` where position is in
    [0, 1].  The returned array is suitable for ``pg.ImageItem.setLookupTable()``.
    """
    positions = np.array([c[0] for c in cs], dtype=np.float64)
    colors = np.array(
        [[int(c[1][i : i + 2], 16) for i in (1, 3, 5)] for c in cs],
        dtype=np.float64,
    )

    lut = np.zeros((n_pts, 4), dtype=np.uint8)
    xs = np.linspace(0.0, 1.0, n_pts)
    for ch in range(3):
        lut[:, ch] = np.interp(xs, positions, colors[:, ch]).astype(np.uint8)
    lut[:, 3] = 255
    return lut


# ---------------------------------------------------------------------------
# All layer definitions and info text (shared by both UIs)
# ---------------------------------------------------------------------------

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
        "using shortest-path optimisation. The route avoids steep slopes, "
        "hazardous terrain, and comms dead zones — it will not plan through "
        "areas with no radio line-of-sight to the lander, and strongly "
        "prefers paths that maintain strong signal coverage.",
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
