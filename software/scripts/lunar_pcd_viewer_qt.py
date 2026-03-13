#!/usr/bin/env python3
"""
Perseus Lunar PCD Viewer — Native PyQt5 + pyqtgraph Desktop Application

GPU-accelerated replacement for the Plotly Dash browser dashboard.
All 12 panels with full interactivity, rendered natively via OpenGL.

Usage:
    python3 lunar_pcd_viewer_qt.py <path_to.pcd>
"""

import glob
import importlib.util
import os
import sys

_NIXGL_MARKER = "_PERSEUS_NIXGL_LAUNCHED"


def _setup_nix_gl_env():
    """Fix OpenGL/EGL, Qt platform, and Python deps inside a Nix shell.

    Nix shells isolate the dynamic linker from the host's GPU drivers.
    This function:
    1. Adds missing Python packages (pyqtgraph, PyOpenGL) to PYTHONPATH
       by discovering them in /nix/store if they're not already importable.
    2. Sets QT_PLUGIN_PATH to include both qtbase and qtwayland plugin
       dirs discovered from the Nix store.
    3. Forces QT_QPA_PLATFORM=wayland on Wayland sessions (Qt5 on GNOME
       ignores XDG_SESSION_TYPE=wayland unless explicitly told).
    4. Discovers the nixGL wrapper in /nix/store and re-execs through it
       so Mesa GL/EGL libraries and vendor configs are properly available.

    On the first invocation (no marker env var), the process re-execs
    through nixGL.  The marker prevents infinite re-exec loops.
    """

    # --- 1. Ensure pyqtgraph & PyOpenGL on PYTHONPATH ---
    _nix_pkg_names = {
        "pyqtgraph": "python3.13-pyqtgraph-*",
        "OpenGL": "python3.13-pyopengl-*",
    }
    extra_paths = []
    for mod_name, store_pattern in _nix_pkg_names.items():
        if importlib.util.find_spec(mod_name) is None:
            candidates = sorted(glob.glob(
                f"/nix/store/*-{store_pattern}/lib/python3.*/site-packages"
            ))
            if candidates:
                extra_paths.append(candidates[-1])
    if extra_paths:
        pp = os.environ.get("PYTHONPATH", "")
        new_pp = ":".join(extra_paths) + (":" + pp if pp else "")
        os.environ["PYTHONPATH"] = new_pp
        # Also add to sys.path for the current process
        for p in extra_paths:
            if p not in sys.path:
                sys.path.insert(0, p)

    # --- 2. Qt plugin path (qtbase + qtwayland) ---
    if not os.environ.get("QT_PLUGIN_PATH"):
        try:
            qt_ver = __import__("PyQt5.QtCore", fromlist=["qVersion"]).qVersion()
        except Exception:
            qt_ver = "5.15"
        qt_plugin_dirs = sorted(
            glob.glob(f"/nix/store/*-qtbase-{qt_ver}-bin/lib/qt-*/plugins")
        ) + sorted(
            glob.glob(f"/nix/store/*-qtwayland-{qt_ver}-bin/lib/qt-*/plugins")
        )
        qt_plugin_dirs = [d for d in qt_plugin_dirs
                          if os.path.isdir(os.path.join(d, "platforms"))]
        if qt_plugin_dirs:
            os.environ["QT_PLUGIN_PATH"] = ":".join(qt_plugin_dirs)

    # --- 3. Force Wayland platform on Wayland sessions ---
    if (os.environ.get("XDG_SESSION_TYPE") == "wayland"
            and not os.environ.get("QT_QPA_PLATFORM")):
        os.environ["QT_QPA_PLATFORM"] = "wayland"

    # --- 4. Re-exec through nixGL for Mesa GL/EGL drivers ---
    if os.environ.get(_NIXGL_MARKER):
        return  # Already launched through nixGL

    nixgl_bins = sorted(glob.glob("/nix/store/*-nixGL/bin/nixGL"))
    if not nixgl_bins:
        return  # nixGL not available
    nixgl = nixgl_bins[-1]

    os.environ[_NIXGL_MARKER] = "1"
    argv = [nixgl, sys.executable] + sys.argv
    print("[PERSEUS] Re-launching through nixGL for GPU driver access...",
          flush=True)
    os.execve(nixgl, argv, os.environ)


# Must run BEFORE importing PyQt5/pyqtgraph so the dynamic linker sees GL libs
_setup_nix_gl_env()

import argparse
import math
import time
from datetime import datetime, timezone
from functools import partial

import numpy as np

from PyQt5.QtCore import Qt, QTimer, QThread, pyqtSignal, QRectF
from PyQt5.QtGui import QColor, QFont, QPalette
from PyQt5.QtWidgets import (
    QApplication,
    QMainWindow,
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QRadioButton,
    QButtonGroup,
    QSlider,
    QDateEdit,
    QSpinBox,
    QFrame,
    QScrollArea,
    QStackedWidget,
    QStatusBar,
    QSplitter,
    QGroupBox,
    QGridLayout,
    QSizePolicy,
)

import pyqtgraph as pg
import pyqtgraph.opengl as gl

from lunar_pcd_compute import (
    load_pcd,
    make_terrain_grid,
    DEFAULT_LAT,
    DEFAULT_LON,
    compute_sun_direction,
    compute_shadow_map,
    compute_slope_map,
    compute_hazard_map,
    generate_ice_deposits,
    compute_line_of_sight,
    compute_comms_coverage,
    compute_lunar_cycle_illumination,
    compute_traversal_cost,
    build_sparse_graph,
    find_path,
    compute_rover_footprint,
    compute_mission_score,
    compute_energy_cost_grid,
    compute_battery_range,
    compute_wp_route,
    _world_to_grid,
    colorscale_to_lut,
    BATTERY_ENERGY_WH,
    LUNAR_CS,
    TOPO_CS,
    SHADOW_CS,
    HAZARD_CS,
    COMMS_CS,
    ICE_CS,
    SCORE_CS,
    SOLAR_RAD_CS,
    RANGE_CS,
    THEMES,
    ALL_LAYERS,
    LAYER_INFO,
)


# ---------------------------------------------------------------------------
# QThread workers for expensive computations
# ---------------------------------------------------------------------------


class CommsWorker(QThread):
    finished = pyqtSignal(object)

    def __init__(self, xg, yg, zg, base_pos):
        super().__init__()
        self.xg, self.yg, self.zg = xg, yg, zg
        self.base_pos = base_pos

    def run(self):
        result = compute_comms_coverage(self.xg, self.yg, self.zg, self.base_pos)
        self.finished.emit(result)


class PathWorker(QThread):
    finished = pyqtSignal(object, object)

    def __init__(self, graph, cost_grid, xg, yg, start_xy, end_xy):
        super().__init__()
        self.graph, self.cost_grid = graph, cost_grid
        self.xg, self.yg = xg, yg
        self.start_xy, self.end_xy = start_xy, end_xy

    def run(self):
        path, cost = find_path(
            self.graph, self.cost_grid, self.xg, self.yg,
            self.start_xy, self.end_xy,
        )
        self.finished.emit(path, cost)


class RangeWorker(QThread):
    finished = pyqtSignal(object, object, object)

    def __init__(self, energy_graph, rows, cols, xg, yg, lander_pos, charge_pct):
        super().__init__()
        self.energy_graph = energy_graph
        self.rows, self.cols = rows, cols
        self.xg, self.yg = xg, yg
        self.lander_pos = lander_pos
        self.charge_pct = charge_pct

    def run(self):
        cost, reachable, range_pct = compute_battery_range(
            self.energy_graph, self.rows, self.cols,
            self.xg, self.yg, self.lander_pos, self.charge_pct,
        )
        self.finished.emit(cost, reachable, range_pct)


class WpRouteWorker(QThread):
    finished = pyqtSignal(object, object)

    def __init__(self, lander_pos, waypoints, graph, cost_grid, xg, yg):
        super().__init__()
        self.lander_pos = lander_pos
        self.waypoints = waypoints
        self.graph, self.cost_grid = graph, cost_grid
        self.xg, self.yg = xg, yg

    def run(self):
        path, cost = compute_wp_route(
            self.lander_pos, self.waypoints,
            self.graph, self.cost_grid, self.xg, self.yg,
        )
        self.finished.emit(path, cost)


# ---------------------------------------------------------------------------
# Helper: build a 2D heatmap panel (ImageItem inside a PlotWidget)
# ---------------------------------------------------------------------------


def _make_heatmap_widget(data, lut, x_range, y_range, parent=None):
    """Create a PlotWidget with an ImageItem showing *data* coloured by *lut*."""
    pw = pg.PlotWidget(parent=parent)
    pw.setAspectLocked(True)
    pw.getAxis("bottom").setLabel("X (m)")
    pw.getAxis("left").setLabel("Y (m)")

    img = pg.ImageItem()
    img.setImage(data.T)  # pyqtgraph: data[col, row]
    img.setLookupTable(lut)
    rect = QRectF(x_range[0], y_range[0],
                  x_range[1] - x_range[0], y_range[1] - y_range[0])
    img.setRect(rect)
    pw.addItem(img)
    pw.setRange(xRange=x_range, yRange=y_range, padding=0.02)
    return pw, img


def _norm(data, vmin=None, vmax=None):
    """Normalise data to [0, 255] uint8 for LUT-based rendering."""
    if vmin is None:
        vmin = np.nanmin(data)
    if vmax is None:
        vmax = np.nanmax(data)
    rng = vmax - vmin
    if rng < 1e-12:
        rng = 1.0
    out = ((np.clip(data, vmin, vmax) - vmin) / rng * 255).astype(np.uint8)
    return out


# ---------------------------------------------------------------------------
# Dark / Light stylesheets
# ---------------------------------------------------------------------------


def _build_stylesheet(theme_name):
    t = THEMES.get(theme_name, THEMES["dark"])
    return f"""
    QMainWindow, QWidget {{
        background-color: {t['page_bg']};
        color: {t['font_color']};
        font-family: 'Courier New', monospace;
        font-size: 11px;
    }}
    QLabel {{
        color: {t['font_color']};
    }}
    QLabel#title {{
        color: {t['accent']};
        font-size: 20px;
        font-weight: bold;
        letter-spacing: 3px;
    }}
    QLabel#mode-title {{
        color: {t['accent']};
        font-size: 20px;
        font-weight: bold;
        letter-spacing: 2px;
    }}
    QLabel#mode-desc {{
        color: {t['font_color']};
        font-size: 13px;
    }}
    QLabel#section-header {{
        color: {t['accent']};
        font-weight: bold;
        font-size: 11px;
        letter-spacing: 2px;
    }}
    QLabel#stats {{
        color: {t['muted']};
        font-size: 11px;
    }}
    QPushButton {{
        background-color: {t['btn_bg']};
        color: {t['font_color']};
        border: 1px solid {t['font_color']};
        padding: 4px 12px;
        font-family: 'Courier New', monospace;
        font-weight: bold;
        font-size: 11px;
    }}
    QPushButton:hover {{
        background-color: {t['accent']};
        color: {t['panel_bg']};
    }}
    QRadioButton {{
        color: {t['font_color']};
        font-size: 11px;
        spacing: 4px;
    }}
    QFrame#sidebar {{
        background-color: {t['panel_bg']};
        border: 1px solid {t['grid_color']};
        border-radius: 4px;
    }}
    QFrame#mode-box {{
        background-color: {t['panel_bg']};
        border: 1px solid {t['accent']};
        border-radius: 4px;
        padding: 8px;
    }}
    QFrame#header-bar {{
        border-bottom: 1px solid {t['accent']};
        padding-bottom: 4px;
    }}
    QFrame#sun-bar {{
        background-color: {t['panel_bg']};
        border: 1px solid {t['grid_color']};
        border-radius: 4px;
        padding: 4px;
    }}
    QFrame#info-box {{
        background-color: {t['page_bg']};
        border: 1px solid {t['grid_color']};
        border-radius: 3px;
        padding: 4px;
    }}
    QGroupBox {{
        color: {t['accent']};
        font-weight: bold;
        border: 1px solid {t['grid_color']};
        border-radius: 3px;
        margin-top: 6px;
        padding-top: 10px;
    }}
    QGroupBox::title {{
        subcontrol-origin: margin;
        left: 6px;
        padding: 0 3px;
    }}
    QSlider::groove:horizontal {{
        height: 4px;
        background: {t['grid_color']};
    }}
    QSlider::handle:horizontal {{
        background: {t['accent']};
        width: 12px;
        margin: -4px 0;
        border-radius: 6px;
    }}
    QSpinBox, QDateEdit {{
        background-color: {t['input_bg']};
        color: {t['font_color']};
        border: 1px solid {t['grid_color']};
        padding: 2px 4px;
        font-family: 'Courier New', monospace;
        font-size: 11px;
    }}
    QStatusBar {{
        background-color: {t['panel_bg']};
        color: {t['muted']};
        font-size: 10px;
    }}
    QScrollArea {{
        border: none;
    }}
    """


# ---------------------------------------------------------------------------
# Main window
# ---------------------------------------------------------------------------


class LunarViewerWindow(QMainWindow):
    def __init__(self, pcd_path):
        super().__init__()
        self.setWindowTitle("PERSEUS - Lunar Terrain Monitor")
        self.setMinimumSize(1200, 800)

        self._theme = "dark"
        self._workers = []  # prevent GC of running QThreads

        # ── Load and precompute ──
        self._precompute(pcd_path)

        # ── Build UI ──
        self._build_ui()

        # ── Apply theme ──
        self.setStyleSheet(_build_stylesheet(self._theme))
        self._apply_pg_theme()

        # ── Timers ──
        self._rot_timer = QTimer(self)
        self._rot_timer.timeout.connect(self._rotate_3d)
        self._rot_timer.start(100)
        self._rot_angle = 0.0

        self._cycle_timer = QTimer(self)
        self._cycle_timer.timeout.connect(self._cycle_tick)
        self._cycle_playing = False
        self._cycle_frame = 0

        # Select initial panel
        self._select_layer(0)

    # ------------------------------------------------------------------
    # Precomputation (same pipeline as the Dash app)
    # ------------------------------------------------------------------

    def _precompute(self, pcd_path):
        t0 = time.monotonic()
        print(f"[PERSEUS] Loading point cloud: {pcd_path}")
        self.points, self.intensity = load_pcd(pcd_path)
        n = len(self.points)
        print(f"[PERSEUS] Loaded {n:,} points")

        self.n_points = n
        pts = self.points
        self.x_range = (float(pts[:, 0].min()), float(pts[:, 0].max()))
        self.y_range = (float(pts[:, 1].min()), float(pts[:, 1].max()))
        self.z_range = (float(pts[:, 2].min()), float(pts[:, 2].max()))

        print("[PERSEUS] Interpolating terrain grid...")
        self.xg, self.yg, self.zg = make_terrain_grid(pts, resolution=120)

        now = datetime.now(timezone.utc)
        self._now = now
        sun0 = compute_sun_direction(now)
        self.illum0 = compute_shadow_map(self.xg, self.yg, self.zg, sun0)
        self._sun0 = sun0

        print("[PERSEUS] [1/8] Slope + hazard...")
        self.slope_deg, self.variance = compute_slope_map(self.xg, self.yg, self.zg)
        self.hazard = compute_hazard_map(self.slope_deg, self.variance)

        print("[PERSEUS] [2/8] 28-day illumination cycle...")
        (self.illum_stack, self.cycle_ts, self.solar_uptime,
         self.solar_radiation, self.psr_mask) = compute_lunar_cycle_illumination(
            self.xg, self.yg, self.zg, now, DEFAULT_LAT, DEFAULT_LON, n_steps=28
        )

        print("[PERSEUS] [3/8] Ice deposits...")
        self.ice_prob, self.drill_sites = generate_ice_deposits(
            self.xg, self.yg, self.zg, self.illum0
        )

        self.default_base = (
            (self.x_range[0] + self.x_range[1]) / 2,
            (self.y_range[0] + self.y_range[1]) / 2,
        )
        print("[PERSEUS] [4/8] Comms viewshed...")
        self.comms_coverage = compute_comms_coverage(
            self.xg, self.yg, self.zg, self.default_base
        )

        print("[PERSEUS] [5/8] Traversal cost...")
        self.cost_grid = compute_traversal_cost(
            self.slope_deg, self.hazard, self.solar_uptime, self.comms_coverage
        )

        print("[PERSEUS] [6/8] Mission score...")
        (self.score_xg, self.score_yg, self.score_grid,
         self.score_comp, self.score_summary) = compute_mission_score(
            self.xg, self.yg, self.zg, self.slope_deg,
            self.solar_uptime, self.comms_coverage,
            self.hazard, self.ice_prob, cell_size=0.25,
        )

        print("[PERSEUS] [7/8] Energy cost grid...")
        self.energy_cost_grid = compute_energy_cost_grid(self.slope_deg, self.hazard)

        print("[PERSEUS] [8/8] Sparse graphs...")
        self.grid_rows, self.grid_cols = self.cost_grid.shape
        self.traversal_graph = build_sparse_graph(self.cost_grid, self.xg, self.yg)
        self.energy_graph = build_sparse_graph(self.energy_cost_grid, self.xg, self.yg)

        # Precompute shaded stack for cycle playback
        z_norm = (self.zg - self.zg.min()) / (self.zg.max() - self.zg.min() + 1e-9)
        self.shaded_stack = [z_norm * (0.3 + 0.7 * il) for il in self.illum_stack]

        # Precompute LUTs
        self._luts = {
            "lunar": colorscale_to_lut(LUNAR_CS),
            "topo": colorscale_to_lut(TOPO_CS),
            "shadow": colorscale_to_lut(SHADOW_CS),
            "hazard": colorscale_to_lut(HAZARD_CS),
            "comms": colorscale_to_lut(COMMS_CS),
            "ice": colorscale_to_lut(ICE_CS),
            "score": colorscale_to_lut(SCORE_CS),
            "solar_rad": colorscale_to_lut(SOLAR_RAD_CS),
            "range": colorscale_to_lut(RANGE_CS),
        }

        # Interactive state
        self._base_pos = list(self.default_base)
        self._path_start = None
        self._path_end = None
        self._path_click_state = 0  # 0=waiting start, 1=waiting end, 2=showing path
        self._rover_pos = None
        self._rover_heading = 0
        self._comms_mode = "base"
        self._rover_comms_pos = None
        self._lander_pos = None
        self._range_waypoints = []
        self._range_mode = "lander"
        self._charge_pct = 100
        self._range_cache = {}
        self._range_pct = None
        self._reachable = None

        print(f"[PERSEUS] Total precompute: {time.monotonic() - t0:.1f}s")

    # ------------------------------------------------------------------
    # UI construction
    # ------------------------------------------------------------------

    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setContentsMargins(8, 8, 8, 4)
        root.setSpacing(6)

        # ── Header bar ──
        header = QFrame()
        header.setObjectName("header-bar")
        hlay = QHBoxLayout(header)
        hlay.setContentsMargins(0, 0, 0, 4)

        self._title = QLabel("PERSEUS LUNAR TERRAIN MONITOR")
        self._title.setObjectName("title")
        hlay.addWidget(self._title)
        hlay.addStretch()

        xs = self.x_range[1] - self.x_range[0]
        ys = self.y_range[1] - self.y_range[0]
        zs = self.z_range[1] - self.z_range[0]
        stats = QLabel(
            f"SHACKLETON CRATER   PTS: {self.n_points:,}   "
            f"AREA: {xs:.1f}x{ys:.1f}m   ELEV: {zs:.1f}m"
        )
        stats.setObjectName("stats")
        hlay.addWidget(stats)
        hlay.addStretch()

        self._theme_btn = QPushButton("LIGHT MODE")
        self._theme_btn.clicked.connect(self._toggle_theme)
        hlay.addWidget(self._theme_btn)
        root.addWidget(header)

        # ── Sun / cycle bar ──
        sun_bar = QFrame()
        sun_bar.setObjectName("sun-bar")
        slay = QHBoxLayout(sun_bar)
        slay.setContentsMargins(6, 4, 6, 4)
        slay.setSpacing(6)

        slay.addWidget(QLabel("SUN"))
        slay.addWidget(QLabel("Lat:"))
        self._lat_spin = QSpinBox()
        self._lat_spin.setRange(-90, 90)
        self._lat_spin.setValue(int(DEFAULT_LAT))
        self._lat_spin.setFixedWidth(60)
        slay.addWidget(self._lat_spin)

        slay.addWidget(QLabel("Lon:"))
        self._lon_spin = QSpinBox()
        self._lon_spin.setRange(-180, 180)
        self._lon_spin.setValue(int(DEFAULT_LON))
        self._lon_spin.setFixedWidth(60)
        slay.addWidget(self._lon_spin)

        slay.addWidget(QLabel("Date:"))
        self._date_edit = QDateEdit()
        self._date_edit.setDisplayFormat("yyyy-MM-dd")
        self._date_edit.setDate(self._now.date())
        self._date_edit.setCalendarPopup(True)
        slay.addWidget(self._date_edit)

        slay.addWidget(QLabel("Hour:"))
        self._hour_spin = QSpinBox()
        self._hour_spin.setRange(0, 23)
        self._hour_spin.setValue(self._now.hour)
        self._hour_spin.setFixedWidth(50)
        slay.addWidget(self._hour_spin)

        update_btn = QPushButton("UPDATE")
        update_btn.clicked.connect(self._update_shadow)
        slay.addWidget(update_btn)

        sep = QLabel("|")
        slay.addWidget(sep)

        self._cycle_btn = QPushButton("PLAY CYCLE")
        self._cycle_btn.clicked.connect(self._toggle_cycle)
        slay.addWidget(self._cycle_btn)

        self._sun_info = QLabel("")
        self._sun_info.setObjectName("stats")
        self._update_sun_info(self._sun0)
        slay.addWidget(self._sun_info)
        slay.addStretch()
        root.addWidget(sun_bar)

        # ── Body: sidebar + panels ──
        body = QSplitter(Qt.Horizontal)

        # -- Sidebar --
        sidebar_scroll = QScrollArea()
        sidebar_scroll.setWidgetResizable(True)
        sidebar_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        sidebar_scroll.setMinimumWidth(200)
        sidebar_scroll.setMaximumWidth(220)

        sidebar = QFrame()
        sidebar.setObjectName("sidebar")
        sb_lay = QVBoxLayout(sidebar)
        sb_lay.setContentsMargins(8, 8, 8, 8)
        sb_lay.setSpacing(2)

        lbl = QLabel("LAYERS")
        lbl.setObjectName("section-header")
        sb_lay.addWidget(lbl)

        self._layer_group = QButtonGroup(self)
        self._layer_radios = []
        for i, (key, label) in enumerate(ALL_LAYERS):
            rb = QRadioButton(f" {label}")
            self._layer_group.addButton(rb, i)
            self._layer_radios.append(rb)
            sb_lay.addWidget(rb)
        self._layer_radios[0].setChecked(True)
        self._layer_group.idClicked.connect(self._select_layer)

        self._add_separator(sb_lay)

        # Rover heading
        lbl = QLabel("ROVER")
        lbl.setObjectName("section-header")
        sb_lay.addWidget(lbl)
        sb_lay.addWidget(QLabel("Heading:"))
        self._heading_slider = QSlider(Qt.Horizontal)
        self._heading_slider.setRange(0, 350)
        self._heading_slider.setSingleStep(10)
        self._heading_slider.setValue(0)
        self._heading_slider.valueChanged.connect(self._on_heading_changed)
        sb_lay.addWidget(self._heading_slider)
        self._heading_lbl = QLabel("0 deg")
        sb_lay.addWidget(self._heading_lbl)

        self._add_separator(sb_lay)

        # Comms mode
        lbl = QLabel("COMMS / LOS")
        lbl.setObjectName("section-header")
        sb_lay.addWidget(lbl)
        sb_lay.addWidget(QLabel("Click mode:"))
        self._comms_base_rb = QRadioButton(" Move Base Station")
        self._comms_rover_rb = QRadioButton(" Check Rover LOS")
        self._comms_base_rb.setChecked(True)
        comms_grp = QButtonGroup(self)
        comms_grp.addButton(self._comms_base_rb, 0)
        comms_grp.addButton(self._comms_rover_rb, 1)
        comms_grp.idClicked.connect(
            lambda i: setattr(self, '_comms_mode', 'base' if i == 0 else 'rover')
        )
        sb_lay.addWidget(self._comms_base_rb)
        sb_lay.addWidget(self._comms_rover_rb)

        comms_info = QFrame()
        comms_info.setObjectName("info-box")
        ci_lay = QVBoxLayout(comms_info)
        ci_lay.setContentsMargins(4, 4, 4, 4)
        self._comms_base_lbl = QLabel(
            f"Base: ({self.default_base[0]:.1f}, {self.default_base[1]:.1f})"
        )
        self._comms_rover_lbl = QLabel("Rover: click map")
        ci_lay.addWidget(self._comms_base_lbl)
        ci_lay.addWidget(self._comms_rover_lbl)
        sb_lay.addWidget(comms_info)

        self._add_separator(sb_lay)

        # Battery range
        lbl = QLabel("BATTERY RANGE")
        lbl.setObjectName("section-header")
        sb_lay.addWidget(lbl)
        sb_lay.addWidget(QLabel("Click mode:"))
        self._range_lander_rb = QRadioButton(" Place Lander")
        self._range_wp_rb = QRadioButton(" Add Waypoint")
        self._range_lander_rb.setChecked(True)
        range_grp = QButtonGroup(self)
        range_grp.addButton(self._range_lander_rb, 0)
        range_grp.addButton(self._range_wp_rb, 1)
        range_grp.idClicked.connect(
            lambda i: setattr(self, '_range_mode', 'lander' if i == 0 else 'waypoint')
        )
        sb_lay.addWidget(self._range_lander_rb)
        sb_lay.addWidget(self._range_wp_rb)

        sb_lay.addWidget(QLabel("Battery charge %:"))
        self._charge_slider = QSlider(Qt.Horizontal)
        self._charge_slider.setRange(10, 100)
        self._charge_slider.setSingleStep(5)
        self._charge_slider.setValue(100)
        self._charge_slider.valueChanged.connect(self._on_charge_changed)
        sb_lay.addWidget(self._charge_slider)
        self._charge_lbl = QLabel("100%")
        sb_lay.addWidget(self._charge_lbl)

        range_info = QFrame()
        range_info.setObjectName("info-box")
        ri_lay = QVBoxLayout(range_info)
        ri_lay.setContentsMargins(4, 4, 4, 4)
        ri_lay.addWidget(QLabel("24V 50Ah (1200Wh)"))
        ri_lay.addWidget(QLabel("Idle: 0.5A | Flat: 4A | Max: 40A"))
        ri_lay.addWidget(QLabel("Reserve: 25% (300Wh)"))
        self._range_lander_lbl = QLabel("Lander: not placed")
        self._range_wp_lbl = QLabel("Waypoints: 0")
        ri_lay.addWidget(self._range_lander_lbl)
        ri_lay.addWidget(self._range_wp_lbl)
        sb_lay.addWidget(range_info)

        clear_btn = QPushButton("CLEAR WAYPOINTS")
        clear_btn.clicked.connect(self._clear_waypoints)
        sb_lay.addWidget(clear_btn)

        self._add_separator(sb_lay)

        # Mission score readout
        lbl = QLabel("MISSION SCORE")
        lbl.setObjectName("section-header")
        sb_lay.addWidget(lbl)
        score_box = QFrame()
        score_box.setObjectName("info-box")
        sc_lay = QGridLayout(score_box)
        sc_lay.setContentsMargins(4, 4, 4, 4)
        sc_lay.setSpacing(2)

        self._score_title_lbl = QLabel("HOVER OVER MAP")
        self._score_title_lbl.setStyleSheet("font-weight: bold;")
        sc_lay.addWidget(self._score_title_lbl, 0, 0, 1, 2)
        self._score_coord_lbl = QLabel("X: --  Y: --")
        sc_lay.addWidget(self._score_coord_lbl, 1, 0, 1, 2)

        for i, name in enumerate(["Solar:", "Slope:", "Clearance:", "Comms:"]):
            sc_lay.addWidget(QLabel(name), i + 2, 0)
        self._score_vals = []
        for i in range(4):
            lbl = QLabel("--/25")
            lbl.setAlignment(Qt.AlignRight)
            sc_lay.addWidget(lbl, i + 2, 1)
            self._score_vals.append(lbl)

        sc_lay.addWidget(QLabel("TOTAL:"), 6, 0)
        self._score_total_lbl = QLabel("--/100")
        self._score_total_lbl.setAlignment(Qt.AlignRight)
        self._score_total_lbl.setStyleSheet("font-weight: bold;")
        sc_lay.addWidget(self._score_total_lbl, 6, 1)

        sb_lay.addWidget(score_box)

        sc_sum = self.score_summary
        sb_lay.addWidget(QLabel(
            f"AVG: {sc_sum['mean_score']:.0f} | BEST: {sc_sum['max_score']:.0f} | "
            f">70: {sc_sum['coverage_above_70']:.0f}%"
        ))
        sb_lay.addWidget(QLabel(
            f"Best: ({sc_sum['best_location'][0]:.1f}, "
            f"{sc_sum['best_location'][1]:.1f})"
        ))

        sb_lay.addStretch()
        sidebar_scroll.setWidget(sidebar)
        body.addWidget(sidebar_scroll)

        # -- Right column: mode desc + stacked panels --
        right_col = QWidget()
        rc_lay = QVBoxLayout(right_col)
        rc_lay.setContentsMargins(0, 0, 0, 0)
        rc_lay.setSpacing(6)

        # Mode description box
        mode_box = QFrame()
        mode_box.setObjectName("mode-box")
        mb_lay = QVBoxLayout(mode_box)
        mb_lay.setContentsMargins(10, 8, 10, 8)
        self._mode_title = QLabel()
        self._mode_title.setObjectName("mode-title")
        self._mode_desc = QLabel()
        self._mode_desc.setObjectName("mode-desc")
        self._mode_desc.setWordWrap(True)
        mb_lay.addWidget(self._mode_title)
        mb_lay.addWidget(self._mode_desc)
        rc_lay.addWidget(mode_box)

        # Stacked panel area
        self._stack = QStackedWidget()
        self._panels = {}
        self._build_panels()
        rc_lay.addWidget(self._stack, 1)

        body.addWidget(right_col)
        body.setStretchFactor(0, 0)
        body.setStretchFactor(1, 1)
        root.addWidget(body, 1)

        # Status bar
        self._status = QStatusBar()
        self.setStatusBar(self._status)

    def _add_separator(self, layout):
        line = QFrame()
        line.setFrameShape(QFrame.HLine)
        line.setFrameShadow(QFrame.Sunken)
        layout.addWidget(line)

    # ------------------------------------------------------------------
    # Build all 12 panels
    # ------------------------------------------------------------------

    def _build_panels(self):
        xr = self.x_range
        yr = self.y_range

        for i, (key, label) in enumerate(ALL_LAYERS):
            if key == "3d":
                w = self._build_3d_panel()
            elif key == "elevation":
                w = self._build_heatmap_panel(
                    _norm(self.zg), self._luts["topo"], "elevation"
                )
            elif key == "contour":
                w = self._build_contour_panel()
            elif key == "solar":
                w = self._build_solar_panel()
            elif key == "hazard":
                w = self._build_heatmap_panel(
                    _norm(self.hazard, 0, 1), self._luts["hazard"], "hazard"
                )
            elif key == "path":
                w = self._build_path_panel()
            elif key == "comms":
                w = self._build_comms_panel()
            elif key == "resources":
                w = self._build_resources_panel()
            elif key == "rover":
                w = self._build_rover_panel()
            elif key == "psr":
                w = self._build_psr_panel()
            elif key == "score":
                w = self._build_score_panel()
            elif key == "range":
                w = self._build_range_panel()
            else:
                w = QLabel(f"Panel: {label}")

            self._panels[key] = w
            self._stack.addWidget(w)

    # ── 3D terrain ──
    def _build_3d_panel(self):
        try:
            self._gl_widget = gl.GLViewWidget()
            pts = self.points
            n = len(pts)
            sub = max(1, n // 100_000)  # much more points than Dash could handle
            pts_sub = pts[::sub]
            z = pts_sub[:, 2]

            # Color by elevation using lunar LUT
            z_norm = ((z - z.min()) / (z.max() - z.min() + 1e-9) * 255).astype(int)
            z_norm = np.clip(z_norm, 0, 255)
            lut = self._luts["lunar"]
            colors = lut[z_norm].astype(np.float32) / 255.0

            scatter = gl.GLScatterPlotItem(
                pos=pts_sub, color=colors, size=1.5, pxMode=True
            )
            self._gl_widget.addItem(scatter)

            cx = (self.x_range[0] + self.x_range[1]) / 2
            cy = (self.y_range[0] + self.y_range[1]) / 2
            cz = (self.z_range[0] + self.z_range[1]) / 2
            dist = max(self.x_range[1] - self.x_range[0],
                       self.y_range[1] - self.y_range[0]) * 2
            self._gl_widget.setCameraPosition(
                pos=pg.Vector(cx, cy, cz), distance=dist, elevation=35, azimuth=0
            )
            self._gl_available = True
            return self._gl_widget
        except Exception as e:
            print(f"[PERSEUS] 3D OpenGL panel unavailable: {e}")
            self._gl_available = False
            # Fallback: show elevation heatmap instead
            pw, img = _make_heatmap_widget(
                _norm(self.zg), self._luts["lunar"],
                self.x_range, self.y_range
            )
            fallback_label = pg.TextItem(
                "3D VIEW UNAVAILABLE — OpenGL context failed\n"
                "Try: nixgl python3 lunar_pcd_viewer_qt.py <pcd>",
                color="#ff4400", anchor=(0.5, 0.5)
            )
            fallback_label.setFont(QFont("Courier New", 12))
            fallback_label.setPos(
                (self.x_range[0] + self.x_range[1]) / 2,
                (self.y_range[0] + self.y_range[1]) / 2,
            )
            pw.addItem(fallback_label)
            return pw

    # ── Generic heatmap ──
    def _build_heatmap_panel(self, data_norm, lut, key):
        pw, img = _make_heatmap_widget(data_norm, lut, self.x_range, self.y_range)
        setattr(self, f'_{key}_img', img)
        setattr(self, f'_{key}_pw', pw)

        def _hover(evt, pw=pw, data=data_norm, key=key):
            pos = evt[0]
            mouse_point = pw.plotItem.vb.mapSceneToView(pos)
            x, y = mouse_point.x(), mouse_point.y()
            if (self.x_range[0] <= x <= self.x_range[1] and
                    self.y_range[0] <= y <= self.y_range[1]):
                self._status.showMessage(f"{key.upper()}  X: {x:.2f}m  Y: {y:.2f}m")

        pw.scene().sigMouseMoved.connect(_hover)
        return pw

    # ── Contour panel ──
    def _build_contour_panel(self):
        pw, img = _make_heatmap_widget(
            _norm(self.zg), self._luts["topo"], self.x_range, self.y_range
        )
        self._contour_pw = pw

        # Add isocurve lines
        z_min, z_max = float(self.zg.min()), float(self.zg.max())
        n_contours = 20
        levels = np.linspace(z_min, z_max, n_contours + 2)[1:-1]
        for level in levels:
            iso = pg.IsocurveItem(
                data=self.zg.T, level=level, pen=pg.mkPen("#00ccff", width=1)
            )
            # Scale isocurve to match image coordinates
            rows, cols = self.zg.shape
            sx = (self.x_range[1] - self.x_range[0]) / cols
            sy = (self.y_range[1] - self.y_range[0]) / rows
            iso.setTransform(
                pg.QtGui.QTransform().translate(
                    self.x_range[0], self.y_range[0]
                ).scale(sx, sy)
            )
            pw.addItem(iso)
        return pw

    # ── Solar/shadow panel ──
    def _build_solar_panel(self):
        z_norm = (self.zg - self.zg.min()) / (self.zg.max() - self.zg.min() + 1e-9)
        shaded = z_norm * (0.3 + 0.7 * self.illum0)
        pw, img = _make_heatmap_widget(
            _norm(shaded, 0, 1), self._luts["shadow"], self.x_range, self.y_range
        )
        self._solar_pw = pw
        self._solar_img = img
        self._solar_shaded = shaded
        return pw

    # ── Path planner panel ──
    def _build_path_panel(self):
        pw, img = _make_heatmap_widget(
            _norm(self.hazard, 0, 1), self._luts["hazard"], self.x_range, self.y_range
        )
        img.setOpacity(0.4)
        self._path_pw = pw
        self._path_img = img

        # Overlay items
        self._path_line = pg.PlotCurveItem(pen=pg.mkPen("#00ccff", width=3))
        pw.addItem(self._path_line)
        self._path_start_marker = pg.ScatterPlotItem(
            size=14, pen=pg.mkPen("w", width=2), brush=pg.mkBrush("#00ff88")
        )
        self._path_end_marker = pg.ScatterPlotItem(
            size=14, pen=pg.mkPen("w", width=2), brush=pg.mkBrush("#cc0000")
        )
        pw.addItem(self._path_start_marker)
        pw.addItem(self._path_end_marker)

        self._path_label = pg.TextItem(
            "CLICK TO SET WAYPOINTS", color="#00ccff", anchor=(0.5, 0.5)
        )
        self._path_label.setFont(QFont("Courier New", 14))
        cx = (self.x_range[0] + self.x_range[1]) / 2
        cy = (self.y_range[0] + self.y_range[1]) / 2
        self._path_label.setPos(cx, cy)
        pw.addItem(self._path_label)

        self._path_cost_label = pg.TextItem("", color="#00ccff", anchor=(1, 0))
        self._path_cost_label.setFont(QFont("Courier New", 11))
        self._path_cost_label.setPos(self.x_range[1], self.y_range[1])
        pw.addItem(self._path_cost_label)

        pw.scene().sigMouseClicked.connect(self._on_path_click)
        return pw

    # ── Comms panel ──
    def _build_comms_panel(self):
        pw, img = _make_heatmap_widget(
            _norm(self.comms_coverage, 0, 1), self._luts["comms"],
            self.x_range, self.y_range
        )
        self._comms_pw = pw
        self._comms_img = img

        # Base marker
        self._comms_base_marker = pg.ScatterPlotItem(
            size=16, symbol='d', pen=pg.mkPen("w", width=2),
            brush=pg.mkBrush("#00ccff")
        )
        self._comms_base_marker.setData(
            [self.default_base[0]], [self.default_base[1]]
        )
        pw.addItem(self._comms_base_marker)

        self._comms_base_text = pg.TextItem(
            "BASE", color="#00ccff", anchor=(0.5, 1.2)
        )
        self._comms_base_text.setFont(QFont("Courier New", 10))
        self._comms_base_text.setPos(self.default_base[0], self.default_base[1])
        pw.addItem(self._comms_base_text)

        # Rover marker
        self._comms_rover_marker = pg.ScatterPlotItem(
            size=12, symbol='o', pen=pg.mkPen("w", width=1),
            brush=pg.mkBrush("#00ff88")
        )
        pw.addItem(self._comms_rover_marker)

        # LOS line segments
        self._comms_los_lines = []

        pw.scene().sigMouseClicked.connect(self._on_comms_click)
        return pw

    # ── Resources / ice panel ──
    def _build_resources_panel(self):
        pw, img = _make_heatmap_widget(
            _norm(self.ice_prob * 100, 0, 100), self._luts["ice"],
            self.x_range, self.y_range
        )
        self._resources_pw = pw

        # Drill site markers
        if self.drill_sites:
            ds = np.asarray(self.drill_sites)
            scatter = pg.ScatterPlotItem(
                pos=ds, size=14, symbol='star', pen=pg.mkPen("w", width=1),
                brush=pg.mkBrush("#ff00ff")
            )
            pw.addItem(scatter)
        return pw

    # ── Rover sim panel ──
    def _build_rover_panel(self):
        pw, img = _make_heatmap_widget(
            _norm(self.slope_deg), self._luts["hazard"], self.x_range, self.y_range
        )
        self._rover_pw = pw
        self._rover_img = img

        # Rover body outline
        self._rover_body = pg.PlotCurveItem(
            pen=pg.mkPen("#00ccff", width=2), fillLevel=None
        )
        pw.addItem(self._rover_body)

        # Rover body fill
        self._rover_fill = pg.FillBetweenItem(
            pg.PlotCurveItem(), pg.PlotCurveItem(),
            brush=pg.mkBrush(0, 204, 255, 60)
        )

        # Wheel markers
        self._rover_wheels = pg.ScatterPlotItem(
            size=10, pen=pg.mkPen("w", width=1)
        )
        pw.addItem(self._rover_wheels)

        # Info text
        self._rover_info_text = pg.TextItem("", color="#00ccff", anchor=(0.5, 1.5))
        self._rover_info_text.setFont(QFont("Courier New", 10))
        pw.addItem(self._rover_info_text)

        self._rover_prompt = pg.TextItem(
            "CLICK TO PLACE ROVER", color="#00ccff", anchor=(0.5, 0.5)
        )
        self._rover_prompt.setFont(QFont("Courier New", 14))
        cx = (self.x_range[0] + self.x_range[1]) / 2
        cy = (self.y_range[0] + self.y_range[1]) / 2
        self._rover_prompt.setPos(cx, cy)
        pw.addItem(self._rover_prompt)

        pw.scene().sigMouseClicked.connect(self._on_rover_click)
        return pw

    # ── PSR / solar radiation panel ──
    def _build_psr_panel(self):
        rad_kwh = self.solar_radiation / 1000.0
        pw, img = _make_heatmap_widget(
            _norm(rad_kwh), self._luts["solar_rad"], self.x_range, self.y_range
        )
        self._psr_pw = pw

        # PSR boundary contour
        psr_float = self.psr_mask.astype(np.float32)
        iso = pg.IsocurveItem(
            data=psr_float.T, level=0.5, pen=pg.mkPen("#ff00ff", width=2)
        )
        rows, cols = psr_float.shape
        sx = (self.x_range[1] - self.x_range[0]) / cols
        sy = (self.y_range[1] - self.y_range[0]) / rows
        iso.setTransform(
            pg.QtGui.QTransform().translate(
                self.x_range[0], self.y_range[0]
            ).scale(sx, sy)
        )
        pw.addItem(iso)

        # Stats annotation
        psr_count = int(np.sum(self.psr_mask))
        psr_pct = psr_count / (rows * cols) * 100
        rad_max = float(np.max(rad_kwh)) if np.any(rad_kwh > 0) else 1.0
        txt = pg.TextItem(
            f"PSR: {psr_count:,} ({psr_pct:.1f}%) | Peak: {rad_max:.0f} kWh/m²",
            color="#ff00ff", anchor=(1, 0)
        )
        txt.setFont(QFont("Courier New", 10))
        txt.setPos(self.x_range[1], self.y_range[1])
        pw.addItem(txt)
        return pw

    # ── Mission score panel ──
    def _build_score_panel(self):
        score_x_range = (
            float(self.score_xg[0, 0]), float(self.score_xg[0, -1])
        )
        score_y_range = (
            float(self.score_yg[0, 0]), float(self.score_yg[-1, 0])
        )
        pw, img = _make_heatmap_widget(
            _norm(self.score_grid, 0, 100), self._luts["score"],
            score_x_range, score_y_range
        )
        self._score_pw = pw
        self._score_img = img

        def _score_hover(evt, pw=pw):
            pos = evt[0]
            mouse_point = pw.plotItem.vb.mapSceneToView(pos)
            x, y = mouse_point.x(), mouse_point.y()
            if (score_x_range[0] <= x <= score_x_range[1] and
                    score_y_range[0] <= y <= score_y_range[1]):
                # Find nearest score cell
                ci = int(np.clip(
                    np.round((x - score_x_range[0]) /
                             (score_x_range[1] - score_x_range[0] + 1e-12) *
                             (self.score_grid.shape[1] - 1)),
                    0, self.score_grid.shape[1] - 1
                ))
                ri = int(np.clip(
                    np.round((y - score_y_range[0]) /
                             (score_y_range[1] - score_y_range[0] + 1e-12) *
                             (self.score_grid.shape[0] - 1)),
                    0, self.score_grid.shape[0] - 1
                ))
                solar_v = self.score_comp["solar"][ri, ci]
                slope_v = self.score_comp["slope"][ri, ci]
                clear_v = self.score_comp["clearance"][ri, ci]
                comms_v = self.score_comp["comms"][ri, ci]
                total_v = self.score_grid[ri, ci]

                self._score_coord_lbl.setText(f"X: {x:.2f}m  Y: {y:.2f}m")
                self._score_title_lbl.setText(f"SITE SCORE: {total_v:.0f}/100")
                self._score_vals[0].setText(f"{solar_v:.1f}/25")
                self._score_vals[1].setText(f"{slope_v:.1f}/25")
                self._score_vals[2].setText(f"{clear_v:.1f}/25")
                self._score_vals[3].setText(f"{comms_v:.1f}/25")
                self._score_total_lbl.setText(f"{total_v:.0f}/100")
                self._status.showMessage(
                    f"SCORE  X: {x:.2f}m  Y: {y:.2f}m  "
                    f"Total: {total_v:.0f}/100"
                )

        pw.scene().sigMouseMoved.connect(_score_hover)
        return pw

    # ── Battery range panel ──
    def _build_range_panel(self):
        # Start with hazard background
        pw, img = _make_heatmap_widget(
            _norm(self.hazard, 0, 1), self._luts["hazard"],
            self.x_range, self.y_range
        )
        img.setOpacity(0.5)
        self._range_pw = pw
        self._range_bg_img = img

        # Range heatmap (hidden initially — needs dummy data so setRect works)
        rows, cols = self.zg.shape
        self._range_img = pg.ImageItem(np.zeros((cols, rows), dtype=np.uint8))
        self._range_img.setLookupTable(self._luts["range"])
        self._range_img.setRect(QRectF(
            self.x_range[0], self.y_range[0],
            self.x_range[1] - self.x_range[0],
            self.y_range[1] - self.y_range[0]
        ))
        self._range_img.setVisible(False)
        pw.addItem(self._range_img)

        # Lander marker
        self._range_lander_marker = pg.ScatterPlotItem(
            size=18, symbol='d', pen=pg.mkPen("w", width=2),
            brush=pg.mkBrush("#ffaa00")
        )
        pw.addItem(self._range_lander_marker)

        self._range_lander_text = pg.TextItem(
            "LANDER", color="#ffaa00", anchor=(0.5, 1.2)
        )
        self._range_lander_text.setFont(QFont("Courier New", 10))
        self._range_lander_text.setVisible(False)
        pw.addItem(self._range_lander_text)

        # Waypoint markers
        self._range_wp_scatter = pg.ScatterPlotItem(
            size=12, symbol='o', pen=pg.mkPen("w", width=2),
            brush=pg.mkBrush("#00ccff")
        )
        pw.addItem(self._range_wp_scatter)

        # Route line
        self._range_route_line = pg.PlotCurveItem(
            pen=pg.mkPen("#00ccff", width=3)
        )
        pw.addItem(self._range_route_line)

        # Cost label
        self._range_cost_label = pg.TextItem("", color="#00ccff", anchor=(1, 1))
        self._range_cost_label.setFont(QFont("Courier New", 11))
        self._range_cost_label.setPos(self.x_range[1], self.y_range[0])
        pw.addItem(self._range_cost_label)

        # Charge info
        self._range_charge_label = pg.TextItem(
            f"CHARGE: 100% (1200 Wh) | RESERVE: 25% (300 Wh)",
            color="#00ff88", anchor=(0.5, 0)
        )
        self._range_charge_label.setFont(QFont("Courier New", 10))
        self._range_charge_label.setPos(
            (self.x_range[0] + self.x_range[1]) / 2, self.y_range[1]
        )
        pw.addItem(self._range_charge_label)

        # Prompt
        self._range_prompt = pg.TextItem(
            "CLICK TO PLACE LANDER", color="#ffaa00", anchor=(0.5, 0.5)
        )
        self._range_prompt.setFont(QFont("Courier New", 14))
        self._range_prompt.setPos(
            (self.x_range[0] + self.x_range[1]) / 2,
            (self.y_range[0] + self.y_range[1]) / 2
        )
        pw.addItem(self._range_prompt)

        pw.scene().sigMouseClicked.connect(self._on_range_click)
        return pw

    # ------------------------------------------------------------------
    # Layer switching
    # ------------------------------------------------------------------

    def _select_layer(self, idx):
        key = ALL_LAYERS[idx][0]
        self._stack.setCurrentIndex(idx)
        title, desc = LAYER_INFO.get(key, ("", ""))
        self._mode_title.setText(title)
        self._mode_desc.setText(desc)

    # ------------------------------------------------------------------
    # Theme toggle
    # ------------------------------------------------------------------

    def _toggle_theme(self):
        self._theme = "light" if self._theme == "dark" else "dark"
        self._theme_btn.setText(
            "DARK MODE" if self._theme == "light" else "LIGHT MODE"
        )
        self.setStyleSheet(_build_stylesheet(self._theme))
        self._apply_pg_theme()

    def _apply_pg_theme(self):
        t = THEMES[self._theme]
        bg = QColor(t["page_bg"])
        fg = QColor(t["font_color"])
        # Update all PlotWidgets
        for attr_name in dir(self):
            obj = getattr(self, attr_name, None)
            if isinstance(obj, pg.PlotWidget):
                obj.setBackground(bg)
                for axis_name in ("bottom", "left"):
                    ax = obj.getAxis(axis_name)
                    ax.setPen(pg.mkPen(fg))
                    ax.setTextPen(pg.mkPen(fg))
        # GL widget
        if getattr(self, '_gl_available', False) and hasattr(self, '_gl_widget'):
            r, g, b = bg.redF(), bg.greenF(), bg.blueF()
            self._gl_widget.setBackgroundColor((r, g, b, 1.0))

    # ------------------------------------------------------------------
    # Sun / shadow
    # ------------------------------------------------------------------

    def _update_sun_info(self, sun_dir):
        el = math.degrees(math.asin(np.clip(sun_dir[2], -1, 1)))
        az = math.degrees(math.atan2(sun_dir[0], sun_dir[1])) % 360
        st = "ABOVE HORIZON" if sun_dir[2] > 0 else "BELOW HORIZON"
        pct = float(np.mean(self.illum0) * 100)
        self._sun_info.setText(
            f"Az: {az:.1f} | Elev: {el:.2f} | {st} | {pct:.0f}% lit"
        )

    def _update_shadow(self):
        d = self._date_edit.date()
        dt = datetime(d.year(), d.month(), d.day(),
                      self._hour_spin.value(), tzinfo=timezone.utc)
        lat = self._lat_spin.value()
        lon = self._lon_spin.value()

        sd = compute_sun_direction(dt, float(lat), float(lon))
        il = compute_shadow_map(self.xg, self.yg, self.zg, sd)

        z_norm = (self.zg - self.zg.min()) / (self.zg.max() - self.zg.min() + 1e-9)
        shaded = z_norm * (0.3 + 0.7 * il)
        self._solar_img.setImage(_norm(shaded, 0, 1).T)
        self.illum0 = il
        self._sun0 = sd
        self._update_sun_info(sd)

    # ------------------------------------------------------------------
    # Cycle playback
    # ------------------------------------------------------------------

    def _toggle_cycle(self):
        self._cycle_playing = not self._cycle_playing
        if self._cycle_playing:
            self._cycle_timer.start(400)
            self._cycle_btn.setText("STOP CYCLE")
        else:
            self._cycle_timer.stop()
            self._cycle_btn.setText("PLAY CYCLE")

    def _cycle_tick(self):
        self._cycle_frame = (self._cycle_frame + 1) % len(self.illum_stack)
        frame = self._cycle_frame
        shaded = self.shaded_stack[frame]
        self._solar_img.setImage(_norm(shaded, 0, 1).T)

        ts = self.cycle_ts[frame]
        pct = float(np.mean(self.illum_stack[frame]) * 100)
        self._sun_info.setText(
            f"CYCLE DAY {frame * 28.0 / len(self.illum_stack):.1f}/28 | "
            f"{ts.strftime('%Y-%m-%d %H:%M')} UTC | {pct:.0f}% lit"
        )

    # ------------------------------------------------------------------
    # 3D auto-rotation
    # ------------------------------------------------------------------

    def _rotate_3d(self):
        self._rot_angle = (self._rot_angle + 0.4) % 360
        if getattr(self, '_gl_available', False) and hasattr(self, '_gl_widget'):
            self._gl_widget.setCameraPosition(azimuth=self._rot_angle)

    # ------------------------------------------------------------------
    # Path planner interaction
    # ------------------------------------------------------------------

    def _on_path_click(self, evt):
        # Only respond if path panel is active
        if self._stack.currentWidget() is not self._panels.get("path"):
            return
        if evt.button() != Qt.LeftButton:
            return

        pos = self._path_pw.plotItem.vb.mapSceneToView(evt.scenePos())
        x, y = pos.x(), pos.y()
        if not (self.x_range[0] <= x <= self.x_range[1] and
                self.y_range[0] <= y <= self.y_range[1]):
            return

        if self._path_click_state == 0:
            # Set start
            self._path_start = (x, y)
            self._path_end = None
            self._path_start_marker.setData([x], [y])
            self._path_end_marker.setData([], [])
            self._path_line.setData([], [])
            self._path_label.setText("CLICK TO SET END POINT")
            self._path_cost_label.setText("")
            self._path_click_state = 1
        elif self._path_click_state == 1:
            # Set end, compute path
            self._path_end = (x, y)
            self._path_end_marker.setData([x], [y])
            self._path_label.setText("COMPUTING...")

            worker = PathWorker(
                self.traversal_graph, self.cost_grid,
                self.xg, self.yg, self._path_start, self._path_end
            )
            worker.finished.connect(self._on_path_result)
            self._workers.append(worker)
            worker.start()
            self._path_click_state = 2
        else:
            # Reset — new start
            self._path_start = (x, y)
            self._path_end = None
            self._path_start_marker.setData([x], [y])
            self._path_end_marker.setData([], [])
            self._path_line.setData([], [])
            self._path_label.setText("CLICK TO SET END POINT")
            self._path_cost_label.setText("")
            self._path_click_state = 1

    def _on_path_result(self, path_coords, total_cost):
        if path_coords is not None:
            pc = np.asarray(path_coords)
            self._path_line.setData(pc[:, 0], pc[:, 1])
            self._path_label.setText("")
            self._path_cost_label.setText(f"ENERGY: {total_cost:.1f}")
        else:
            self._path_label.setText("NO SAFE PATH FOUND")
            self._path_label.setColor(QColor("#cc0000"))
            self._path_cost_label.setText("")

    # ------------------------------------------------------------------
    # Comms interaction
    # ------------------------------------------------------------------

    def _on_comms_click(self, evt):
        if self._stack.currentWidget() is not self._panels.get("comms"):
            return
        if evt.button() != Qt.LeftButton:
            return

        pos = self._comms_pw.plotItem.vb.mapSceneToView(evt.scenePos())
        x, y = pos.x(), pos.y()
        if not (self.x_range[0] <= x <= self.x_range[1] and
                self.y_range[0] <= y <= self.y_range[1]):
            return

        if self._comms_mode == "base":
            self._base_pos = [x, y]
            self._comms_base_marker.setData([x], [y])
            self._comms_base_text.setPos(x, y)
            self._comms_base_lbl.setText(f"Base: ({x:.1f}, {y:.1f})")
            self._comms_rover_lbl.setText("Rover: click map")
            self._comms_rover_marker.setData([], [])
            # Clear LOS lines
            for line in self._comms_los_lines:
                self._comms_pw.removeItem(line)
            self._comms_los_lines.clear()

            # Recompute coverage in thread
            worker = CommsWorker(self.xg, self.yg, self.zg, (x, y))
            worker.finished.connect(self._on_comms_coverage_result)
            self._workers.append(worker)
            worker.start()
        else:
            # Place rover, compute LOS
            self._rover_comms_pos = (x, y)
            self._comms_rover_marker.setData([x], [y])
            base = tuple(self._base_pos)
            los_data = compute_line_of_sight(self.xg, self.yg, self.zg, base, (x, y))
            status = "CLEAR" if los_data["visible"] else "BLOCKED"
            self._comms_rover_lbl.setText(f"Rover: ({x:.1f}, {y:.1f}) [{status}]")

            # Draw LOS line segments
            for line in self._comms_los_lines:
                self._comms_pw.removeItem(line)
            self._comms_los_lines.clear()

            bm = los_data["blocked_mask"]
            n_seg = len(bm)
            lx = np.linspace(base[0], x, n_seg + 1)
            ly = np.linspace(base[1], y, n_seg + 1)
            for i in range(0, n_seg, 3):
                color = "#cc0000" if bm[i] else "#00ff88"
                end = min(i + 4, n_seg)
                line = pg.PlotCurveItem(
                    [lx[i], lx[end]], [ly[i], ly[end]],
                    pen=pg.mkPen(color, width=3)
                )
                self._comms_pw.addItem(line)
                self._comms_los_lines.append(line)

    def _on_comms_coverage_result(self, coverage):
        self.comms_coverage = coverage
        self._comms_img.setImage(_norm(coverage, 0, 1).T)

    # ------------------------------------------------------------------
    # Rover sim interaction
    # ------------------------------------------------------------------

    def _on_rover_click(self, evt):
        if self._stack.currentWidget() is not self._panels.get("rover"):
            return
        if evt.button() != Qt.LeftButton:
            return

        pos = self._rover_pw.plotItem.vb.mapSceneToView(evt.scenePos())
        x, y = pos.x(), pos.y()
        if not (self.x_range[0] <= x <= self.x_range[1] and
                self.y_range[0] <= y <= self.y_range[1]):
            return

        self._rover_pos = (x, y)
        self._update_rover_display()

    def _on_heading_changed(self, val):
        self._rover_heading = val
        self._heading_lbl.setText(f"{val} deg")
        if self._rover_pos is not None:
            self._update_rover_display()

    def _update_rover_display(self):
        if self._rover_pos is None:
            return

        data = compute_rover_footprint(
            self.xg, self.yg, self.zg,
            self._rover_pos, self._rover_heading
        )
        bc = np.asarray(data["body_corners"])
        bx = list(bc[:, 0]) + [bc[0, 0]]
        by = list(bc[:, 1]) + [bc[0, 1]]
        self._rover_body.setData(bx, by)

        wp = np.asarray(data["wheel_positions"])
        contact = data["contact_ok"]
        brushes = [pg.mkBrush("#00ff88" if c else "#cc0000") for c in contact]
        self._rover_wheels.setData(
            pos=wp[:, :2], brush=brushes
        )

        clr = data["min_clearance"]
        clr_color = "#00ff88" if clr > 0.05 else "#ffaa00" if clr > 0 else "#cc0000"
        cx = float(np.mean(bc[:, 0]))
        cy = float(np.mean(bc[:, 1]))
        self._rover_info_text.setPos(cx, cy)
        self._rover_info_text.setText(
            f"CLR: {clr:.2f}m | P: {data['tilt_pitch']:.1f} R: {data['tilt_roll']:.1f}"
        )
        self._rover_info_text.setColor(QColor(clr_color))
        self._rover_prompt.setText("")

    # ------------------------------------------------------------------
    # Battery range interaction
    # ------------------------------------------------------------------

    def _on_range_click(self, evt):
        if self._stack.currentWidget() is not self._panels.get("range"):
            return
        if evt.button() != Qt.LeftButton:
            return

        pos = self._range_pw.plotItem.vb.mapSceneToView(evt.scenePos())
        x, y = pos.x(), pos.y()
        if not (self.x_range[0] <= x <= self.x_range[1] and
                self.y_range[0] <= y <= self.y_range[1]):
            return

        if self._range_mode == "lander":
            self._lander_pos = (x, y)
            self._range_cache.clear()
            self._range_lander_marker.setData([x], [y])
            self._range_lander_text.setPos(x, y)
            self._range_lander_text.setVisible(True)
            self._range_prompt.setText("")
            self._range_lander_lbl.setText(f"Lander: ({x:.1f}, {y:.1f})")
            self._compute_range()
        else:
            # Add waypoint
            self._range_waypoints.append((x, y))
            self._range_wp_lbl.setText(f"Waypoints: {len(self._range_waypoints)}")
            self._update_range_waypoint_display()
            if self._lander_pos is not None:
                self._compute_wp_route()

    def _on_charge_changed(self, val):
        self._charge_pct = val
        self._charge_lbl.setText(f"{val}%")
        wh = BATTERY_ENERGY_WH * val / 100
        self._range_charge_label.setText(
            f"CHARGE: {val}% ({wh:.0f} Wh) | RESERVE: 25% (300 Wh)"
        )
        if self._lander_pos is not None:
            self._compute_range()

    def _clear_waypoints(self):
        self._range_waypoints.clear()
        self._range_wp_scatter.setData([], [])
        self._range_route_line.setData([], [])
        self._range_cost_label.setText("")
        self._range_wp_lbl.setText("Waypoints: 0")

    def _compute_range(self):
        if self._lander_pos is None:
            return
        worker = RangeWorker(
            self.energy_graph, self.grid_rows, self.grid_cols,
            self.xg, self.yg, self._lander_pos, self._charge_pct
        )
        worker.finished.connect(self._on_range_result)
        self._workers.append(worker)
        worker.start()

    def _on_range_result(self, cost_to_reach, reachable, range_pct):
        self._reachable = reachable
        self._range_pct = range_pct

        # Show range heatmap
        display = np.clip(range_pct, 0, 100).copy()
        display[~reachable] = 0
        self._range_img.setImage(_norm(display, 0, 100).T)
        self._range_img.setVisible(True)
        self._range_bg_img.setOpacity(0.15)

        # Recompute wp route if waypoints exist
        if self._range_waypoints:
            self._compute_wp_route()

    def _update_range_waypoint_display(self):
        if self._range_waypoints:
            wp = np.asarray(self._range_waypoints)
            self._range_wp_scatter.setData(pos=wp)
        else:
            self._range_wp_scatter.setData([], [])

    def _compute_wp_route(self):
        if self._lander_pos is None or not self._range_waypoints:
            return
        worker = WpRouteWorker(
            self._lander_pos, self._range_waypoints,
            self.traversal_graph, self.cost_grid, self.xg, self.yg
        )
        worker.finished.connect(self._on_wp_route_result)
        self._workers.append(worker)
        worker.start()

    def _on_wp_route_result(self, path, cost):
        if path is not None:
            pp = np.asarray(path)
            self._range_route_line.setData(pp[:, 0], pp[:, 1])
            self._range_cost_label.setText(f"ROUTE: {cost:.1f} Wh")
        else:
            self._range_route_line.setData([], [])
            self._range_cost_label.setText("NO ROUTE FOUND")

    # ------------------------------------------------------------------
    # Keyboard shortcuts
    # ------------------------------------------------------------------

    def keyPressEvent(self, event):
        key = event.key()

        # 1-9, 0 for layers
        layer_keys = {
            Qt.Key_1: 0, Qt.Key_2: 1, Qt.Key_3: 2,
            Qt.Key_4: 3, Qt.Key_5: 4, Qt.Key_6: 5,
            Qt.Key_7: 6, Qt.Key_8: 7, Qt.Key_9: 8,
            Qt.Key_0: 9,
        }
        if key in layer_keys:
            idx = layer_keys[key]
            if idx < len(ALL_LAYERS):
                self._layer_radios[idx].setChecked(True)
                self._select_layer(idx)
            return

        # Minus and Equals for layers 10 and 11
        if key == Qt.Key_Minus and len(ALL_LAYERS) > 10:
            self._layer_radios[10].setChecked(True)
            self._select_layer(10)
            return
        if key == Qt.Key_Equal and len(ALL_LAYERS) > 11:
            self._layer_radios[11].setChecked(True)
            self._select_layer(11)
            return

        # T for theme
        if key == Qt.Key_T:
            self._toggle_theme()
            return

        # Space for play/pause cycle
        if key == Qt.Key_Space:
            self._toggle_cycle()
            return

        super().keyPressEvent(event)


# ---------------------------------------------------------------------------
# CLI entry point
# ---------------------------------------------------------------------------


def main():
    parser = argparse.ArgumentParser(description="Perseus Lunar PCD Viewer (Qt)")
    parser.add_argument("pcd_file", help="Path to a .pcd point cloud file")
    args = parser.parse_args()

    app = QApplication(sys.argv)
    app.setApplicationName("PERSEUS Lunar Terrain Monitor")

    # Set default monospace font
    font = QFont("Courier New", 11)
    app.setFont(font)

    print(f"\n{'=' * 60}")
    print("  PERSEUS LUNAR TERRAIN MONITOR (Qt)")
    print("  Shackleton Crater — South Pole")
    print(f"{'=' * 60}\n")

    window = LunarViewerWindow(args.pcd_file)
    window.showMaximized()

    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
