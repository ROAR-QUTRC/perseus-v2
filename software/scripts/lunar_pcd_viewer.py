#!/usr/bin/env python3
"""
Perseus Lunar PCD Viewer — NASA-style Mission Control Dashboard

Visualises LiDAR point cloud scans (.pcd) as if monitoring a lunar rover
near Shackleton Crater on the Moon's South Pole.

Panels:
  1. Rotating 3D terrain view (~35 deg elevation, interactive mouse control)
  2. Top-down colour-coded elevation heat map
  3. Topographic contour map with annotated elevation lines
  4. Solar illumination / shadow map for a user-selected date & time

Usage:
    python3 lunar_pcd_viewer.py <path_to.pcd>
    python3 lunar_pcd_viewer.py scan_1.pcd --port 8060
"""

import argparse
import math
from datetime import datetime, timezone

import numpy as np
from scipy.interpolate import griddata
from scipy.ndimage import gaussian_filter

import plotly.graph_objects as go
from dash import Dash, html, dcc, Input, Output, State


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
    """Sun direction as a unit vector [east, north, up] in a local tangent frame
    at the given selenographic (lat, lon) position on the Moon.

    Model overview:
      - The sub-solar longitude sweeps 360 deg over one synodic month (~29.53 d).
      - The sub-solar latitude oscillates +/-1.5427 deg (lunar obliquity) with
        the nodical month (~27.21 d).
      - From the observer's (lat, lon) we compute the angular separation to the
        sub-solar point to get local elevation and azimuth.
    """
    if dt.tzinfo is None:
        dt = dt.replace(tzinfo=timezone.utc)

    ref_new_moon = datetime(2000, 1, 6, 18, 14, tzinfo=timezone.utc)
    days_since = (dt - ref_new_moon).total_seconds() / 86400.0

    # Sub-solar longitude: full sweep per synodic month
    synodic = 29.530588
    phase = (days_since % synodic) / synodic
    subsolar_lon = (phase * 360.0) % 360.0  # degrees

    # Sub-solar latitude: oscillates with nodical month
    nodical = 27.2122
    lib_phase = (days_since % nodical) / nodical
    moon_tilt = 1.5427  # degrees
    subsolar_lat = moon_tilt * math.sin(2 * math.pi * lib_phase)

    # Convert to radians
    obs_lat = math.radians(lat_deg)
    obs_lon = math.radians(lon_deg)
    ss_lat = math.radians(subsolar_lat)
    ss_lon = math.radians(subsolar_lon)

    # Angular distance (great-circle) between observer and sub-solar point
    dlon = ss_lon - obs_lon
    cos_c = math.sin(obs_lat) * math.sin(ss_lat) + math.cos(obs_lat) * math.cos(
        ss_lat
    ) * math.cos(dlon)
    cos_c = max(-1.0, min(1.0, cos_c))

    # Sun elevation = 90 - angular_distance  (sun is at zenith at sub-solar point)
    elevation = math.asin(cos_c)  # radians above local horizon

    # Azimuth of the sun from observer (measured from north, clockwise)
    sin_c = math.sqrt(1 - cos_c * cos_c) + 1e-12
    cos_az = (math.sin(ss_lat) - math.sin(obs_lat) * cos_c) / (
        math.cos(obs_lat) * sin_c + 1e-12
    )
    sin_az = math.cos(ss_lat) * math.sin(dlon) / sin_c
    azimuth = math.atan2(sin_az, cos_az)

    # Convert to ENU unit vector
    ce = math.cos(elevation)
    return np.array(
        [
            ce * math.sin(azimuth),  # East
            ce * math.cos(azimuth),  # North
            math.sin(elevation),  # Up
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


# ---------------------------------------------------------------------------
# Theme constants
# ---------------------------------------------------------------------------

DARK_BG = "#0a0a12"
PANEL_BG = "#0d1117"
GRID_CLR = "#1a2332"
TEXT_CLR = "#00ff88"
ACCENT = "#00ccff"

LAYOUT_BASE = dict(
    paper_bgcolor=PANEL_BG,
    plot_bgcolor=PANEL_BG,
    font=dict(family="Courier New, monospace", size=11, color=TEXT_CLR),
    margin=dict(l=40, r=40, t=45, b=35),
)


# ---------------------------------------------------------------------------
# Figure builders (pure functions — no Dash dependency)
# ---------------------------------------------------------------------------


def fig_3d(points, intensity, subsample, camera_eye):
    """Build the 3D scatter terrain figure."""
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
                        title=dict(text="Elev (m)", font=dict(color=TEXT_CLR, size=11)),
                        len=0.5,
                        thickness=12,
                        x=0.98,
                        tickfont=dict(color=TEXT_CLR, size=10),
                    ),
                ),
                hovertemplate="X: %{x:.2f}m<br>Y: %{y:.2f}m<br>Z: %{z:.2f}m<extra></extra>",
            )
        ]
    )
    fig.update_layout(
        **LAYOUT_BASE,
        scene=dict(
            xaxis=dict(
                title="X (m)",
                gridcolor=GRID_CLR,
                showbackground=True,
                backgroundcolor=PANEL_BG,
                color=TEXT_CLR,
                zerolinecolor=GRID_CLR,
            ),
            yaxis=dict(
                title="Y (m)",
                gridcolor=GRID_CLR,
                showbackground=True,
                backgroundcolor=PANEL_BG,
                color=TEXT_CLR,
                zerolinecolor=GRID_CLR,
            ),
            zaxis=dict(
                title="Z (m)",
                gridcolor=GRID_CLR,
                showbackground=True,
                backgroundcolor=PANEL_BG,
                color=TEXT_CLR,
                zerolinecolor=GRID_CLR,
            ),
            camera=dict(
                eye=camera_eye, center=dict(x=0, y=0, z=0), up=dict(x=0, y=0, z=1)
            ),
            aspectmode="data",
        ),
        uirevision="terrain",  # preserves user camera across data updates
    )
    return fig


def fig_heatmap(xg, yg, zg):
    fig = go.Figure(
        data=[
            go.Heatmap(
                x=xg[0, :],
                y=yg[:, 0],
                z=zg,
                colorscale=TOPO_CS,
                colorbar=dict(
                    title=dict(text="Elev (m)", font=dict(color=TEXT_CLR, size=11)),
                    len=0.45,
                    thickness=12,
                    tickfont=dict(color=TEXT_CLR, size=10),
                ),
                hovertemplate="X: %{x:.2f}m<br>Y: %{y:.2f}m<br>Elev: %{z:.2f}m<extra></extra>",
            )
        ]
    )
    fig.update_layout(
        **LAYOUT_BASE,
        xaxis=dict(
            title="X (m)",
            scaleanchor="y",
            gridcolor=GRID_CLR,
            color=TEXT_CLR,
            zerolinecolor=GRID_CLR,
        ),
        yaxis=dict(
            title="Y (m)", gridcolor=GRID_CLR, color=TEXT_CLR, zerolinecolor=GRID_CLR
        ),
    )
    return fig


def fig_contour(xg, yg, zg, n_contours=20):
    fig = go.Figure(
        data=[
            go.Contour(
                x=xg[0, :],
                y=yg[:, 0],
                z=zg,
                ncontours=n_contours,
                contours=dict(
                    showlabels=True,
                    labelfont=dict(size=9, color="#00ffaa"),
                    coloring="heatmap",
                ),
                colorscale=TOPO_CS,
                colorbar=dict(
                    title=dict(text="Elev (m)", font=dict(color=TEXT_CLR, size=11)),
                    len=0.45,
                    thickness=12,
                    tickfont=dict(color=TEXT_CLR, size=10),
                ),
                line=dict(width=1.5, color="rgba(0,255,170,0.5)"),
                hovertemplate="X: %{x:.2f}m<br>Y: %{y:.2f}m<br>Elev: %{z:.2f}m<extra></extra>",
            )
        ]
    )
    fig.update_layout(
        **LAYOUT_BASE,
        xaxis=dict(
            title="X (m)",
            scaleanchor="y",
            gridcolor=GRID_CLR,
            color=TEXT_CLR,
            zerolinecolor=GRID_CLR,
        ),
        yaxis=dict(
            title="Y (m)", gridcolor=GRID_CLR, color=TEXT_CLR, zerolinecolor=GRID_CLR
        ),
    )
    return fig


def fig_shadow(xg, yg, zg, illum):
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
    fig.update_layout(
        **LAYOUT_BASE,
        xaxis=dict(
            title="X (m)",
            scaleanchor="y",
            gridcolor=GRID_CLR,
            color=TEXT_CLR,
            zerolinecolor=GRID_CLR,
        ),
        yaxis=dict(
            title="Y (m)", gridcolor=GRID_CLR, color=TEXT_CLR, zerolinecolor=GRID_CLR
        ),
    )
    return fig


# ---------------------------------------------------------------------------
# Dash application
# ---------------------------------------------------------------------------

# Client-side JS that rotates the 3D camera.  Runs entirely in the browser
# so there is zero server round-trip for the animation.  If the user grabs
# the scene with the mouse, Plotly's built-in orbit handler takes over and
# the auto-rotation pauses until the interval fires again.
ROTATION_JS = """
function(n, fig) {
    if (!fig || !fig.layout) { return window.dash_clientside.no_update; }
    var angle = (n * 0.4) % 360;          // degrees per tick
    var rad   = angle * Math.PI / 180;
    var elev  = 35 * Math.PI / 180;       // 35-deg look-down
    var r     = 1.8;                       // camera distance factor
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
    xg, yg, zg = make_terrain_grid(points, resolution=250)

    now = datetime.now(timezone.utc)
    sun0 = compute_sun_direction(now)
    illum0 = compute_shadow_map(xg, yg, zg, sun0)

    # Stats for header
    xr = [float(points[:, 0].min()), float(points[:, 0].max())]
    yr = [float(points[:, 1].min()), float(points[:, 1].max())]
    zr = [float(points[:, 2].min()), float(points[:, 2].max())]
    xs, ys, zs = xr[1] - xr[0], yr[1] - yr[0], zr[1] - zr[0]

    # Pre-build static figures (built once, never rebuilt by server)
    init_eye = dict(
        x=1.8 * math.cos(math.radians(35)),
        y=0,
        z=1.8 * math.sin(math.radians(35)),
    )
    fig3d_init = fig_3d(points, intensity, sub, init_eye)
    fig_hm = fig_heatmap(xg, yg, zg)
    fig_ct = fig_contour(xg, yg, zg)
    fig_sh = fig_shadow(xg, yg, zg, illum0)

    elev0 = math.degrees(math.asin(np.clip(sun0[2], -1, 1)))
    az0 = math.degrees(math.atan2(sun0[0], sun0[1])) % 360
    status0 = "ABOVE HORIZON" if sun0[2] > 0 else "BELOW HORIZON"
    info0 = f"Sun Az: {az0:.1f} | Elev: {elev0:.2f} | {status0} | {np.mean(illum0) * 100:.0f}% illuminated"

    print("[PERSEUS] Building dashboard...")

    _input_style = {
        "backgroundColor": "#1a1a2e",
        "color": TEXT_CLR,
        "border": f"1px solid {GRID_CLR}",
        "padding": "4px 8px",
        "fontFamily": "Courier New, monospace",
        "fontSize": "12px",
        "width": "90px",
    }

    app = Dash(__name__)
    app.title = "PERSEUS — Lunar Terrain Monitor"

    def _panel(label, graph_id, figure, extra_children=None):
        children = [
            html.Div(
                label,
                style={
                    "position": "absolute",
                    "top": "4px",
                    "left": "10px",
                    "fontSize": "10px",
                    "color": ACCENT,
                    "zIndex": 10,
                    "letterSpacing": "2px",
                },
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
            style={
                "border": f"1px solid {GRID_CLR}",
                "borderRadius": "4px",
                "position": "relative",
            },
            children=children,
        )

    app.layout = html.Div(
        style={
            "backgroundColor": DARK_BG,
            "minHeight": "100vh",
            "padding": "10px",
            "fontFamily": "Courier New, monospace",
            "color": TEXT_CLR,
        },
        children=[
            # ── Header ──
            html.Div(
                style={
                    "textAlign": "center",
                    "borderBottom": f"1px solid {ACCENT}",
                    "paddingBottom": "8px",
                    "marginBottom": "10px",
                },
                children=[
                    html.H1(
                        "PERSEUS LUNAR TERRAIN MONITOR",
                        style={
                            "color": ACCENT,
                            "margin": "0",
                            "fontSize": "22px",
                            "letterSpacing": "4px",
                        },
                    ),
                    html.Div(
                        [
                            html.Span(
                                "SHACKLETON CRATER — SOUTH POLE",
                                style={"fontSize": "12px", "marginRight": "30px"},
                            ),
                            html.Span(
                                f"POINTS: {n:,}",
                                style={"fontSize": "12px", "marginRight": "30px"},
                            ),
                            html.Span(
                                f"AREA: {xs:.1f}m x {ys:.1f}m",
                                style={"fontSize": "12px", "marginRight": "30px"},
                            ),
                            html.Span(
                                f"ELEV RANGE: {zs:.1f}m", style={"fontSize": "12px"}
                            ),
                        ]
                    ),
                ],
            ),
            # ── Sun control bar ──
            html.Div(
                style={
                    "display": "flex",
                    "alignItems": "center",
                    "justifyContent": "center",
                    "gap": "12px",
                    "padding": "8px",
                    "backgroundColor": PANEL_BG,
                    "border": f"1px solid {GRID_CLR}",
                    "borderRadius": "4px",
                    "marginBottom": "10px",
                    "flexWrap": "wrap",
                },
                children=[
                    html.Label(
                        "SUN SIMULATION",
                        style={
                            "color": ACCENT,
                            "fontWeight": "bold",
                            "fontSize": "13px",
                        },
                    ),
                    html.Label("Lat:", style={"fontSize": "12px"}),
                    dcc.Input(
                        id="moon-lat",
                        type="number",
                        value=DEFAULT_LAT,
                        step=0.1,
                        style=_input_style,
                        placeholder="Lat (-90..90)",
                    ),
                    html.Label("Lon:", style={"fontSize": "12px"}),
                    dcc.Input(
                        id="moon-lon",
                        type="number",
                        value=DEFAULT_LON,
                        step=0.1,
                        style=_input_style,
                        placeholder="Lon (-180..180)",
                    ),
                    html.Label("Date:", style={"fontSize": "12px"}),
                    dcc.Input(
                        id="sun-date",
                        type="date",
                        value=now.strftime("%Y-%m-%d"),
                        style=_input_style,
                    ),
                    html.Label("UTC Hour:", style={"fontSize": "12px"}),
                    dcc.Slider(
                        id="sun-hour",
                        min=0,
                        max=23,
                        step=1,
                        value=now.hour,
                        marks={
                            h: {
                                "label": f"{h:02d}",
                                "style": {"color": TEXT_CLR, "fontSize": "9px"},
                            }
                            for h in range(0, 24, 3)
                        },
                        tooltip={"placement": "bottom", "always_visible": False},
                    ),
                    html.Button(
                        "UPDATE SUN",
                        id="sun-btn",
                        style={
                            "backgroundColor": "#1a3322",
                            "color": TEXT_CLR,
                            "border": f"1px solid {TEXT_CLR}",
                            "padding": "6px 16px",
                            "cursor": "pointer",
                            "fontFamily": "Courier New, monospace",
                            "fontSize": "12px",
                            "fontWeight": "bold",
                        },
                    ),
                    html.Span(
                        id="sun-info",
                        children=info0,
                        style={
                            "fontSize": "11px",
                            "color": "#aaa",
                            "minWidth": "220px",
                        },
                    ),
                ],
            ),
            # ── 2x2 panel grid ──
            html.Div(
                style={
                    "display": "grid",
                    "gridTemplateColumns": "1fr 1fr",
                    "gridTemplateRows": "1fr 1fr",
                    "gap": "8px",
                    "height": "calc(100vh - 180px)",
                },
                children=[
                    _panel(
                        "3D TERRAIN — 35 DEG OBLIQUE VIEW  [drag to orbit]",
                        "terrain-3d",
                        fig3d_init,
                        extra_children=[
                            dcc.Interval(id="rot-tick", interval=100, n_intervals=0)
                        ],
                    ),
                    _panel("ELEVATION MAP — TOP-DOWN", "topo-heatmap", fig_hm),
                    _panel("TOPOGRAPHIC CONTOUR MAP", "contour-map", fig_ct),
                    _panel("SOLAR ILLUMINATION MAP", "shadow-map", fig_sh),
                ],
            ),
            # Hidden store for base 3D figure (data-only, no camera) so the
            # client-side callback can animate camera without re-fetching data.
            dcc.Store(id="fig3d-store", data=fig3d_init.to_dict()),
        ],
    )

    # ── Client-side rotation callback (no server round-trip) ──
    app.clientside_callback(
        ROTATION_JS,
        Output("terrain-3d", "figure"),
        Input("rot-tick", "n_intervals"),
        State("fig3d-store", "data"),
    )

    # ── Shadow / sun callback (server-side, only on button click) ──
    @app.callback(
        Output("shadow-map", "figure"),
        Output("sun-info", "children"),
        Input("sun-btn", "n_clicks"),
        State("sun-date", "value"),
        State("sun-hour", "value"),
        State("moon-lat", "value"),
        State("moon-lon", "value"),
        prevent_initial_call=True,
    )
    def update_shadow(_, date_str, hour, lat, lon):
        try:
            dt = datetime.strptime(date_str, "%Y-%m-%d").replace(
                hour=int(hour),
                tzinfo=timezone.utc,
            )
        except Exception:
            dt = now

        lat_v = float(lat) if lat is not None else DEFAULT_LAT
        lon_v = float(lon) if lon is not None else DEFAULT_LON
        lat_v = max(-90.0, min(90.0, lat_v))
        lon_v = max(-180.0, min(180.0, lon_v))

        sd = compute_sun_direction(dt, lat_v, lon_v)
        il = compute_shadow_map(xg, yg, zg, sd)
        pct = float(np.mean(il) * 100)

        el = math.degrees(math.asin(np.clip(sd[2], -1, 1)))
        az = math.degrees(math.atan2(sd[0], sd[1])) % 360
        st = "ABOVE HORIZON" if sd[2] > 0 else "BELOW HORIZON"
        info = (
            f"Pos: {lat_v:.1f}N {lon_v:.1f}E | "
            f"Sun Az: {az:.1f} | Elev: {el:.2f} | {st} | {pct:.0f}% illuminated"
        )

        return fig_shadow(xg, yg, zg, il), info

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
