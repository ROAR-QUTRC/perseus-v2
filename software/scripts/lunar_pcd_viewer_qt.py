#!/usr/bin/env python3
"""
Perseus Lunar PCD Viewer — Native Qt Desktop Application

PyQt5 + pyqtgraph desktop viewer for lunar LiDAR point cloud data.
All computation reuses lunar_pcd_compute.py (shared with the Dash browser viewer).

Usage:
    nixgl python3 lunar_pcd_viewer_qt.py <path_to.pcd>
    nixgl python3 lunar_pcd_viewer_qt.py scan_1.pcd
"""

import argparse
import math
import sys
import time
from datetime import datetime, timedelta, timezone
from functools import partial

# ---------------------------------------------------------------------------
# PyOpenGL context fix — must be applied before any OpenGL imports.
# PyOpenGL's contextdata.getContext() cannot detect QOpenGLWidget contexts,
# causing "no valid context" errors on every GL call that stores pointers.
# Patching it to return a fallback context ID fixes rendering.
# ---------------------------------------------------------------------------
from OpenGL import contextdata as _ctxdata

_orig_getContext = _ctxdata.getContext


def _patched_getContext(context=None):
    try:
        return _orig_getContext(context)
    except Exception:
        return 0


_ctxdata.getContext = _patched_getContext

import numpy as np

from PyQt5.QtCore import (
    QObject,
    Qt,
    QThread,
    QTimer,
    pyqtSignal,
)
from PyQt5.QtGui import QColor, QFont
from PyQt5.QtWidgets import (
    QApplication,
    QButtonGroup,
    QDateEdit,
    QDoubleSpinBox,
    QFrame,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QListWidget,
    QListWidgetItem,
    QMainWindow,
    QPushButton,
    QRadioButton,
    QScrollArea,
    QSlider,
    QSpinBox,
    QSplitter,
    QVBoxLayout,
    QWidget,
)
from PyQt5.QtCore import QDate

import pyqtgraph as pg
import pyqtgraph.opengl as gl

from lunar_pcd_compute import (
    ALL_LAYERS,
    BATTERY_ENERGY_WH,
    COMMS_CS,
    DEFAULT_LAT,
    DEFAULT_LON,
    HAZARD_CS,
    ICE_CS,
    LAYER_INFO,
    LUNAR_CS,
    RANGE_CS,
    SCORE_CS,
    SHADOW_CS,
    SOLAR_RAD_CS,
    THEMES,
    TOPO_CS,
    build_sparse_graph,
    colorscale_to_lut,
    compute_battery_range,
    compute_comms_coverage,
    compute_energy_cost_grid,
    compute_hazard_map,
    compute_line_of_sight,
    compute_lunar_cycle_illumination,
    compute_mission_score,
    compute_rover_footprint,
    compute_shadow_map,
    compute_slope_map,
    compute_sun_direction,
    compute_traversal_cost,
    compute_wp_route,
    find_path,
    generate_ice_deposits,
    load_pcd,
    make_terrain_grid,
    _bilinear_interp,
    _world_to_grid,
)


# ---------------------------------------------------------------------------
# Pre-build LUTs for all colour scales
# ---------------------------------------------------------------------------

LUTS = {
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


# ---------------------------------------------------------------------------
# Theme helper
# ---------------------------------------------------------------------------


def _theme(name):
    return THEMES.get(name, THEMES["dark"])


def _stylesheet(t):
    """Build a QSS stylesheet from a theme dict."""
    return f"""
    QMainWindow, QWidget {{
        background-color: {t['page_bg']};
        color: {t['font_color']};
        font-family: "Courier New", monospace;
        font-size: 11px;
    }}
    QLabel {{
        color: {t['font_color']};
    }}
    QListWidget {{
        background-color: {t['sidebar_bg']};
        color: {t['font_color']};
        border: 1px solid {t['grid_color']};
        outline: none;
    }}
    QListWidget::item {{
        padding: 4px 6px;
    }}
    QListWidget::item:selected {{
        background-color: {t['btn_bg']};
        color: {t['accent']};
    }}
    QPushButton {{
        background-color: {t['btn_bg']};
        color: {t['font_color']};
        border: 1px solid {t['grid_color']};
        padding: 4px 10px;
        font-family: "Courier New", monospace;
    }}
    QPushButton:hover {{
        border-color: {t['accent']};
    }}
    QGroupBox {{
        border: 1px solid {t['grid_color']};
        margin-top: 8px;
        padding-top: 12px;
        font-weight: bold;
        color: {t['accent']};
    }}
    QGroupBox::title {{
        subcontrol-origin: margin;
        left: 6px;
        padding: 0 4px;
    }}
    QSlider::groove:horizontal {{
        height: 4px;
        background: {t['grid_color']};
    }}
    QSlider::handle:horizontal {{
        background: {t['accent']};
        width: 12px;
        margin: -4px 0;
    }}
    QSpinBox {{
        background-color: {t['input_bg']};
        color: {t['font_color']};
        border: 1px solid {t['grid_color']};
        padding: 2px;
    }}
    QRadioButton {{
        color: {t['font_color']};
    }}
    QScrollArea {{
        border: none;
    }}
    QLabel#descBar {{
        background-color: {t['sidebar_bg']};
        border: 1px solid {t['grid_color']};
        padding: 4px 8px;
        color: {t['font_color']};
    }}
    QGroupBox#hoverGroup {{
        border: 1px solid {t['accent']};
        background-color: {t['sidebar_bg']};
    }}
    """


# ---------------------------------------------------------------------------
# Background worker for heavy computation
# ---------------------------------------------------------------------------


class ComputeWorker(QObject):
    finished = pyqtSignal(dict)
    progress = pyqtSignal(str)

    def __init__(self, func, kwargs=None):
        super().__init__()
        self._func = func
        self._kwargs = kwargs or {}

    def run(self):
        result = self._func(**self._kwargs)
        self.finished.emit(result)


# ---------------------------------------------------------------------------
# Main window
# ---------------------------------------------------------------------------


class LunarPCDViewer(QMainWindow):
    def __init__(self, pcd_path: str):
        super().__init__()
        self._pcd_path = pcd_path
        self._theme_name = "dark"
        self._current_layer = "elevation"

        # Interactive state
        self._path_start = None
        self._path_end = None
        self._path_coords = None
        self._path_cost = None
        self._click_mode = None  # "lander", "rover", "wp", or None
        self._base_pos = None
        self._rover_pos_comms = None
        self._los_data = None
        self._rover_pos = None
        self._rover_heading = 0
        self._rover_data = None
        self._lander_pos = None
        self._waypoints = []
        self._charge_pct = 100.0
        self._wp_path = None
        self._wp_cost = None
        self._cycle_frame = 0
        self._cycle_playing = False
        self._range_cache = {}

        # Computed data (filled during init)
        self._points = None
        self._intensity = None
        self._xg = self._yg = self._zg = None
        self._slope_deg = self._variance = None
        self._hazard = None
        self._illum0 = None
        self._illum_stack = None
        self._cycle_ts = None
        self._solar_uptime = None
        self._solar_radiation = None
        self._psr_mask = None
        self._ice_prob = None
        self._drill_sites = None
        self._comms_coverage = None
        self._cost_grid = None
        self._traversal_graph = None
        self._energy_cost_grid = None
        self._energy_graph = None
        self._score_xg = self._score_yg = self._score_grid = None
        self._score_comp = self._score_summary = None
        self._shaded_stack = None
        self._z_norm = None

        # Sun controls state
        self._sun_lat = DEFAULT_LAT
        self._sun_lon = DEFAULT_LON
        self._sun_date = datetime.now(timezone.utc)
        self._sun_hour = self._sun_date.hour

        self._build_ui()
        self._load_data()

    # -----------------------------------------------------------------------
    # UI construction
    # -----------------------------------------------------------------------

    def _build_ui(self):
        self.setWindowTitle("PERSEUS — Lunar PCD Viewer")
        self.resize(1400, 900)

        central = QWidget()
        self.setCentralWidget(central)
        root_layout = QVBoxLayout(central)
        root_layout.setContentsMargins(4, 4, 4, 4)
        root_layout.setSpacing(2)

        # Header
        self._header = QLabel("PERSEUS LUNAR PCD VIEWER — Loading...")
        self._header.setAlignment(Qt.AlignCenter)
        hdr_font = QFont("Courier New", 12, QFont.Bold)
        self._header.setFont(hdr_font)
        root_layout.addWidget(self._header)

        # Stats bar
        self._stats_label = QLabel("")
        self._stats_label.setAlignment(Qt.AlignCenter)
        root_layout.addWidget(self._stats_label)

        # Global lunar lat/long settings bar
        loc_bar = QHBoxLayout()
        loc_bar.addWidget(QLabel("Lunar Lat:"))
        self._global_lat = QDoubleSpinBox()
        self._global_lat.setRange(-90.0, 90.0)
        self._global_lat.setDecimals(1)
        self._global_lat.setSingleStep(0.1)
        self._global_lat.setValue(DEFAULT_LAT)
        self._global_lat.setSuffix(" deg")
        loc_bar.addWidget(self._global_lat)
        loc_bar.addWidget(QLabel("Lon:"))
        self._global_lon = QDoubleSpinBox()
        self._global_lon.setRange(-180.0, 180.0)
        self._global_lon.setDecimals(1)
        self._global_lon.setSingleStep(0.1)
        self._global_lon.setValue(DEFAULT_LON)
        self._global_lon.setSuffix(" deg")
        loc_bar.addWidget(self._global_lon)
        self._btn_apply_location = QPushButton("Apply Location")
        self._btn_apply_location.clicked.connect(self._on_apply_location)
        loc_bar.addWidget(self._btn_apply_location)
        self._location_info = QLabel(
            f"Shackleton Crater ({DEFAULT_LAT}, {DEFAULT_LON})"
        )
        self._location_info.setFont(QFont("Courier New", 9))
        loc_bar.addWidget(self._location_info)
        loc_bar.addStretch()
        root_layout.addLayout(loc_bar)

        # Layer description banner — spans full width above the main content
        self._desc_bar = QLabel("")
        self._desc_bar.setObjectName("descBar")
        self._desc_bar.setWordWrap(True)
        self._desc_bar.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        self._desc_bar.setFont(QFont("Courier New", 14))
        self._desc_bar.setMinimumHeight(70)
        self._desc_bar.setContentsMargins(12, 8, 12, 8)
        root_layout.addWidget(self._desc_bar)

        # Splitter: sidebar | main view
        splitter = QSplitter(Qt.Horizontal)
        root_layout.addWidget(splitter, stretch=1)

        # --- Sidebar ---
        sidebar_scroll = QScrollArea()
        sidebar_scroll.setWidgetResizable(True)
        sidebar_scroll.setMinimumWidth(240)
        sidebar_scroll.setMaximumWidth(300)
        sidebar_widget = QWidget()
        self._sidebar_layout = QVBoxLayout(sidebar_widget)
        self._sidebar_layout.setContentsMargins(4, 4, 4, 4)
        self._sidebar_layout.setSpacing(4)

        # Layer selector
        lbl = QLabel("LAYERS")
        lbl.setFont(QFont("Courier New", 10, QFont.Bold))
        self._sidebar_layout.addWidget(lbl)

        # Layer list — contour merged into elevation, rover sim removed
        self._viewer_layers = [
            (k, l) for k, l in ALL_LAYERS if k not in ("contour", "rover")
        ]
        self._layer_list = QListWidget()
        for key, label in self._viewer_layers:
            item = QListWidgetItem(label)
            item.setData(Qt.UserRole, key)
            self._layer_list.addItem(item)
        self._layer_list.currentRowChanged.connect(self._on_layer_changed)
        self._layer_list.setMaximumHeight(230)
        self._sidebar_layout.addWidget(self._layer_list)

        # Contour overlay toggle (applies to elevation layer)
        from PyQt5.QtWidgets import QCheckBox
        self._chk_contours = QCheckBox("Show contour lines")
        self._chk_contours.setChecked(True)
        self._chk_contours.toggled.connect(self._on_contour_toggled)
        self._sidebar_layout.addWidget(self._chk_contours)

        # ── MISSION SETUP — always visible ──
        setup_group = QGroupBox("MISSION SETUP")
        setup_lay = QVBoxLayout(setup_group)
        setup_lay.setSpacing(4)

        # Lander / comms base
        self._btn_place_lander = QPushButton("Place Lander / Base")
        self._btn_place_lander.setCheckable(True)
        self._btn_place_lander.clicked.connect(
            lambda: self._set_click_mode("lander")
        )
        setup_lay.addWidget(self._btn_place_lander)
        self._lander_label = QLabel("Lander: not placed")
        self._lander_label.setFont(QFont("Courier New", 9))
        self._lander_label.setWordWrap(True)
        setup_lay.addWidget(self._lander_label)

        # Rover
        self._btn_place_rover = QPushButton("Place Rover")
        self._btn_place_rover.setCheckable(True)
        self._btn_place_rover.clicked.connect(
            lambda: self._set_click_mode("rover")
        )
        setup_lay.addWidget(self._btn_place_rover)
        self._rover_label = QLabel("Rover: not placed")
        self._rover_label.setFont(QFont("Courier New", 9))
        self._rover_label.setWordWrap(True)
        setup_lay.addWidget(self._rover_label)

        # Rover heading
        row_h = QHBoxLayout()
        row_h.addWidget(QLabel("Heading:"))
        self._slider_heading = QSlider(Qt.Horizontal)
        self._slider_heading.setRange(0, 359)
        self._slider_heading.setValue(0)
        self._heading_label = QLabel("0 deg")
        self._slider_heading.valueChanged.connect(self._on_heading_changed)
        row_h.addWidget(self._slider_heading)
        row_h.addWidget(self._heading_label)
        setup_lay.addLayout(row_h)

        # Battery charge
        row_c = QHBoxLayout()
        row_c.addWidget(QLabel("Battery:"))
        self._slider_charge = QSlider(Qt.Horizontal)
        self._slider_charge.setRange(10, 100)
        self._slider_charge.setValue(100)
        self._charge_label = QLabel("100%")
        self._slider_charge.valueChanged.connect(self._on_charge_changed)
        row_c.addWidget(self._slider_charge)
        row_c.addWidget(self._charge_label)
        setup_lay.addLayout(row_c)

        # Power consumption model sliders
        # Flat current (A) — draw on easy terrain
        row_flat = QHBoxLayout()
        row_flat.addWidget(QLabel("Flat I:"))
        self._slider_current_flat = QSlider(Qt.Horizontal)
        self._slider_current_flat.setRange(10, 200)  # 1.0 – 20.0 A (x10)
        self._slider_current_flat.setValue(40)  # 4.0 A default
        self._current_flat_label = QLabel("4.0 A")
        self._slider_current_flat.valueChanged.connect(self._on_power_model_changed)
        row_flat.addWidget(self._slider_current_flat)
        row_flat.addWidget(self._current_flat_label)
        setup_lay.addLayout(row_flat)

        # Max current (A) — draw on steep/hazardous terrain
        row_max = QHBoxLayout()
        row_max.addWidget(QLabel("Max I:"))
        self._slider_current_max = QSlider(Qt.Horizontal)
        self._slider_current_max.setRange(50, 800)  # 5.0 – 80.0 A (x10)
        self._slider_current_max.setValue(400)  # 40.0 A default
        self._current_max_label = QLabel("40.0 A")
        self._slider_current_max.valueChanged.connect(self._on_power_model_changed)
        row_max.addWidget(self._slider_current_max)
        row_max.addWidget(self._current_max_label)
        setup_lay.addLayout(row_max)

        # Rover speed (m/s)
        row_spd = QHBoxLayout()
        row_spd.addWidget(QLabel("Speed:"))
        self._slider_speed = QSlider(Qt.Horizontal)
        self._slider_speed.setRange(5, 100)  # 0.05 – 1.00 m/s (x100)
        self._slider_speed.setValue(30)  # 0.30 m/s default
        self._speed_label = QLabel("0.30 m/s")
        self._slider_speed.valueChanged.connect(self._on_power_model_changed)
        row_spd.addWidget(self._slider_speed)
        row_spd.addWidget(self._speed_label)
        setup_lay.addLayout(row_spd)

        # Waypoints
        self._btn_add_wp = QPushButton("Add Waypoint")
        self._btn_add_wp.setCheckable(True)
        self._btn_add_wp.clicked.connect(
            lambda: self._set_click_mode("wp")
        )
        setup_lay.addWidget(self._btn_add_wp)
        self._wp_label = QLabel("Waypoints: 0")
        self._wp_label.setFont(QFont("Courier New", 9))
        setup_lay.addWidget(self._wp_label)
        self._btn_clear_wps = QPushButton("Clear Waypoints")
        self._btn_clear_wps.clicked.connect(self._on_clear_waypoints)
        setup_lay.addWidget(self._btn_clear_wps)

        self._sidebar_layout.addWidget(setup_group)

        # ── LAYER-SPECIFIC CONTROLS ──

        # Sun / shadow controls group
        self._sun_group = QGroupBox("SUN / SHADOW")
        sun_lay = QVBoxLayout(self._sun_group)

        # Lat / Lon
        row = QHBoxLayout()
        row.addWidget(QLabel("Lat:"))
        self._spin_lat = QDoubleSpinBox()
        self._spin_lat.setRange(-90.0, 90.0)
        self._spin_lat.setDecimals(1)
        self._spin_lat.setValue(DEFAULT_LAT)
        row.addWidget(self._spin_lat)
        row.addWidget(QLabel("Lon:"))
        self._spin_lon = QDoubleSpinBox()
        self._spin_lon.setRange(-180.0, 180.0)
        self._spin_lon.setDecimals(1)
        self._spin_lon.setValue(DEFAULT_LON)
        row.addWidget(self._spin_lon)
        sun_lay.addLayout(row)

        # Date picker
        row_date = QHBoxLayout()
        row_date.addWidget(QLabel("Date:"))
        self._date_edit = QDateEdit()
        now = datetime.now(timezone.utc)
        self._date_edit.setDate(QDate(now.year, now.month, now.day))
        self._date_edit.setCalendarPopup(True)
        self._date_edit.setDisplayFormat("yyyy-MM-dd")
        row_date.addWidget(self._date_edit)
        sun_lay.addLayout(row_date)

        # Hour slider
        row_hour = QHBoxLayout()
        row_hour.addWidget(QLabel("Hour:"))
        self._slider_hour = QSlider(Qt.Horizontal)
        self._slider_hour.setRange(0, 23)
        self._slider_hour.setValue(self._sun_hour)
        self._hour_label = QLabel(f"{self._sun_hour:02d}:00")
        self._slider_hour.valueChanged.connect(
            lambda v: self._hour_label.setText(f"{v:02d}:00")
        )
        row_hour.addWidget(self._slider_hour)
        row_hour.addWidget(self._hour_label)
        sun_lay.addLayout(row_hour)

        self._btn_update_sun = QPushButton("Update Shadow Map")
        self._btn_update_sun.clicked.connect(self._on_update_sun)
        sun_lay.addWidget(self._btn_update_sun)

        # Lunar day/night cycle animation (28 Earth days in ~20 seconds)
        self._btn_cycle = QPushButton("Play Lunar Day (20s)")
        self._btn_cycle.clicked.connect(self._toggle_cycle)
        sun_lay.addWidget(self._btn_cycle)
        self._cycle_label = QLabel("")
        self._cycle_label.setFont(QFont("Courier New", 9))
        self._cycle_label.setWordWrap(True)
        sun_lay.addWidget(self._cycle_label)

        self._sidebar_layout.addWidget(self._sun_group)

        # Theme toggle
        self._btn_theme = QPushButton("Toggle Light/Dark")
        self._btn_theme.clicked.connect(self._toggle_theme)
        self._sidebar_layout.addWidget(self._btn_theme)

        # Action detail (path cost, LOS result, etc.)
        self._info_detail = QLabel("")
        self._info_detail.setWordWrap(True)
        self._info_detail.setFont(QFont("Courier New", 10))
        self._sidebar_layout.addWidget(self._info_detail)

        # Real-time hover readout panel
        self._hover_group = QGroupBox("CURSOR READOUT")
        self._hover_group.setObjectName("hoverGroup")
        hover_lay = QVBoxLayout(self._hover_group)
        hover_lay.setContentsMargins(6, 14, 6, 6)
        hover_lay.setSpacing(1)

        self._hover_label = QLabel("Move cursor over terrain...")
        self._hover_label.setWordWrap(True)
        self._hover_label.setFont(QFont("Courier New", 9))
        self._hover_label.setTextFormat(Qt.RichText)
        self._hover_label.setAlignment(Qt.AlignLeft | Qt.AlignTop)
        hover_lay.addWidget(self._hover_label)

        self._sidebar_layout.addWidget(self._hover_group, stretch=1)

        self._sidebar_layout.addStretch()
        sidebar_scroll.setWidget(sidebar_widget)
        splitter.addWidget(sidebar_scroll)

        # --- Main view container ---
        # Use a stacked layout with manual switching instead of
        # QStackedWidget, because QStackedWidget destroys GL contexts.
        from PyQt5.QtWidgets import QStackedLayout

        self._view_container = QWidget()
        self._view_stack = QStackedLayout(self._view_container)
        self._view_stack.setStackingMode(QStackedLayout.StackAll)

        # 2D view (index 0)
        self._plot_widget = pg.PlotWidget()
        self._plot_widget.setAspectLocked(True)
        self._plot_widget.showGrid(x=True, y=True, alpha=0.15)
        self._plot_widget.getPlotItem().getAxis("bottom").setLabel("X (m)")
        self._plot_widget.getPlotItem().getAxis("left").setLabel("Y (m)")
        self._image_item = pg.ImageItem()
        self._plot_widget.addItem(self._image_item)
        self._plot_widget.scene().sigMouseClicked.connect(self._on_plot_clicked)
        self._hover_proxy = pg.SignalProxy(
            self._plot_widget.scene().sigMouseMoved,
            rateLimit=30,
            slot=self._on_plot_hover,
        )
        self._view_stack.addWidget(self._plot_widget)

        # 3D view — lazy-created on first use
        self._gl_view = None
        self._gl_ok = None  # None = not yet tried, True/False = result

        splitter.addWidget(self._view_container)
        splitter.setStretchFactor(0, 0)
        splitter.setStretchFactor(1, 1)

        # Overlay items (reused across layers)
        self._overlay_items = []

        # Cycle timer
        self._cycle_timer = QTimer(self)
        self._cycle_timer.setInterval(300)
        self._cycle_timer.timeout.connect(self._cycle_step)

        # Apply initial theme
        self._apply_theme()

    # -----------------------------------------------------------------------
    # Theme
    # -----------------------------------------------------------------------

    def _apply_theme(self):
        t = _theme(self._theme_name)
        self.setStyleSheet(_stylesheet(t))
        self._plot_widget.setBackground(t["panel_bg"])
        if self._gl_view is not None:
            self._gl_view.setBackgroundColor(t["panel_bg"])
        # Update axis colors
        for axis_name in ("bottom", "left"):
            ax = self._plot_widget.getPlotItem().getAxis(axis_name)
            ax.setPen(pg.mkPen(t["font_color"]))
            ax.setTextPen(pg.mkPen(t["font_color"]))
        self._header.setStyleSheet(f"color: {t['accent']};")
        self._desc_bar.setStyleSheet(
            f"background-color: {t['sidebar_bg']}; "
            f"border: 1px solid {t['grid_color']}; "
            f"color: {t['font_color']}; padding: 4px 8px;"
        )

    def _toggle_theme(self):
        self._theme_name = "light" if self._theme_name == "dark" else "dark"
        self._apply_theme()
        self._show_layer(self._current_layer)

    # -----------------------------------------------------------------------
    # Data loading
    # -----------------------------------------------------------------------

    def _load_data(self):
        """Load PCD and precompute all analysis layers."""
        self._header.setText("PERSEUS — Loading point cloud...")
        QApplication.processEvents()

        t0 = time.monotonic()
        print(f"[PERSEUS] Loading point cloud: {self._pcd_path}")
        self._points, self._intensity = load_pcd(self._pcd_path)
        n = len(self._points)
        print(f"[PERSEUS] Loaded {n:,} points")

        print("[PERSEUS] Interpolating terrain grid...")
        self._xg, self._yg, self._zg = make_terrain_grid(self._points, resolution=120)
        xg, yg, zg = self._xg, self._yg, self._zg

        now = datetime.now(timezone.utc)
        self._sun_date = now
        sun0 = compute_sun_direction(now)
        self._illum0 = compute_shadow_map(xg, yg, zg, sun0)

        self._header.setText("PERSEUS — Computing analysis layers...")
        QApplication.processEvents()

        print("[PERSEUS] [1/8] Slope and hazard...")
        self._slope_deg, self._variance = compute_slope_map(xg, yg, zg)
        self._hazard = compute_hazard_map(self._slope_deg, self._variance)

        print("[PERSEUS] [2/8] 28-day lunar illumination cycle...")
        (
            self._illum_stack,
            self._cycle_ts,
            self._solar_uptime,
            self._solar_radiation,
            self._psr_mask,
        ) = compute_lunar_cycle_illumination(
            xg, yg, zg, now, DEFAULT_LAT, DEFAULT_LON, n_steps=112
        )

        print("[PERSEUS] [3/8] Ice deposits...")
        self._ice_prob, self._drill_sites = generate_ice_deposits(
            xg, yg, zg, self._illum0
        )

        self._base_pos = (
            (float(xg[0, 0]) + float(xg[0, -1])) / 2,
            (float(yg[0, 0]) + float(yg[-1, 0])) / 2,
        )
        self._lander_pos = self._base_pos

        print("[PERSEUS] [4/8] Comms coverage...")
        self._comms_coverage = compute_comms_coverage(xg, yg, zg, self._base_pos)

        print("[PERSEUS] [5/8] Traversal cost grid...")
        self._cost_grid = compute_traversal_cost(
            self._slope_deg, self._hazard, self._solar_uptime, self._comms_coverage
        )

        print("[PERSEUS] [6/8] Mission scores...")
        (
            self._score_xg,
            self._score_yg,
            self._score_grid,
            self._score_comp,
            self._score_summary,
        ) = compute_mission_score(
            xg,
            yg,
            zg,
            self._slope_deg,
            self._solar_uptime,
            self._comms_coverage,
            self._hazard,
            self._ice_prob,
            cell_size=0.25,
        )

        print("[PERSEUS] [7/8] Energy cost grid...")
        self._energy_cost_grid = compute_energy_cost_grid(
            self._slope_deg, self._hazard
        )

        print("[PERSEUS] [8/8] Sparse graphs...")
        self._traversal_graph = build_sparse_graph(self._cost_grid, xg, yg)
        self._energy_graph = build_sparse_graph(self._energy_cost_grid, xg, yg)

        # Precompute shaded stack — only daylight frames (sun above horizon)
        from scipy.ndimage import gaussian_filter as _gf

        self._z_norm = (zg - zg.min()) / (zg.max() - zg.min() + 1e-9)
        self._shaded_stack = []
        self._shaded_ts = []
        for i, il in enumerate(self._illum_stack):
            if float(np.mean(il)) > 0.01:  # sun is up somewhere
                smooth = _gf(il.astype(np.float64), sigma=1.2)
                self._shaded_stack.append(self._z_norm * (0.3 + 0.7 * smooth))
                if self._cycle_ts:
                    self._shaded_ts.append(self._cycle_ts[i])
        if not self._shaded_stack:
            # Fallback: use all frames if no daylight found
            self._shaded_stack = [
                self._z_norm * (0.3 + 0.7 * _gf(il.astype(np.float64), sigma=1.2))
                for il in self._illum_stack
            ]
            self._shaded_ts = list(self._cycle_ts) if self._cycle_ts else []

        elapsed = time.monotonic() - t0
        print(f"[PERSEUS] Total precomputation: {elapsed:.1f}s")

        # Update header with stats
        pts = self._points
        xr = float(pts[:, 0].max()) - float(pts[:, 0].min())
        yr = float(pts[:, 1].max()) - float(pts[:, 1].min())
        zr = float(pts[:, 2].max()) - float(pts[:, 2].min())
        self._header.setText(f"PERSEUS — {self._pcd_path}")
        self._stats_label.setText(
            f"{n:,} points | {xr:.1f} x {yr:.1f} x {zr:.1f} m | "
            f"Grid: {xg.shape[0]}x{xg.shape[1]}"
        )

        # Show initial layer (elevation avoids OpenGL requirement on startup)
        self._update_position_labels()
        self._layer_list.setCurrentRow(1)  # "elevation"
        self._current_layer = "elevation"
        self._show_layer("elevation")
        self._update_info("elevation")
        self._update_sidebar_visibility("elevation")

    # -----------------------------------------------------------------------
    # Layer switching
    # -----------------------------------------------------------------------

    def _on_layer_changed(self, row):
        if row < 0:
            return
        item = self._layer_list.item(row)
        key = item.data(Qt.UserRole)
        self._current_layer = key
        self._show_layer(key)
        self._update_info(key)
        self._update_sidebar_visibility(key)

    def _update_sidebar_visibility(self, layer):
        """Show/hide layer-specific sidebar groups."""
        self._sun_group.setVisible(layer == "solar")

    def _set_click_mode(self, mode):
        """Set what a map click does: 'lander', 'rover', 'wp', or None."""
        # Toggle off if same button pressed again
        if self._click_mode == mode:
            self._click_mode = None
        else:
            self._click_mode = mode

        # Update button checked states
        self._btn_place_lander.setChecked(self._click_mode == "lander")
        self._btn_place_rover.setChecked(self._click_mode == "rover")
        self._btn_add_wp.setChecked(self._click_mode == "wp")

        mode_text = {
            "lander": "Click map to place lander/base station",
            "rover": "Click map to place rover",
            "wp": "Click map to add waypoint",
        }
        self._info_detail.setText(mode_text.get(self._click_mode, ""))

    def _update_position_labels(self):
        """Refresh the lander/rover/WP position labels in the sidebar."""
        if self._base_pos:
            self._lander_label.setText(
                f"Lander: ({self._base_pos[0]:.2f}, {self._base_pos[1]:.2f})"
            )
        else:
            self._lander_label.setText("Lander: not placed")

        if self._rover_pos:
            self._rover_label.setText(
                f"Rover: ({self._rover_pos[0]:.2f}, {self._rover_pos[1]:.2f})"
            )
        else:
            self._rover_label.setText("Rover: not placed")

        self._wp_label.setText(f"Waypoints: {len(self._waypoints)}")

    def _update_info(self, layer):
        title, desc = LAYER_INFO.get(layer, ("", ""))
        t = _theme(self._theme_name)
        self._desc_bar.setText(
            f"<span style='font-size:16pt; color:{t['accent']}; font-weight:bold'>"
            f"{title}</span><br>"
            f"<span style='font-size:12pt'>{desc}</span>"
        )
        self._info_detail.setText("")
        self._hover_label.setText("Move cursor over terrain...")

    def _ensure_gl_view(self):
        """Lazy-create the GLViewWidget on first use."""
        if self._gl_view is not None:
            return True
        if self._gl_ok is False:
            return False  # already tried and failed (not None)
        try:
            view = gl.GLViewWidget()
            view.setCameraPosition(distance=5, elevation=30, azimuth=45)
            t = _theme(self._theme_name)
            view.setBackgroundColor(t["panel_bg"])
            self._view_stack.addWidget(view)  # index 1
            self._gl_view = view
            self._gl_ok = True
            return True
        except Exception as e:
            print(f"[PERSEUS] OpenGL unavailable: {e}")
            self._gl_ok = False
            return False

    def _show_layer(self, layer):
        """Render the selected layer."""
        self._clear_overlays()

        if layer == "3d":
            if self._ensure_gl_view():
                self._plot_widget.setVisible(False)
                self._gl_view.setVisible(True)
                self._gl_view.raise_()
                QApplication.processEvents()
                self._render_3d()
                print(
                    f"[PERSEUS] 3D: size={self._gl_view.width()}x"
                    f"{self._gl_view.height()}, items={len(self._gl_view.items)}"
                )
            else:
                self._info_detail.setText(
                    "3D unavailable — run with nixgl"
                )
        else:
            self._plot_widget.raise_()
            self._plot_widget.setVisible(True)
            if self._gl_view is not None:
                self._gl_view.setVisible(False)
            if layer == "elevation":
                self._render_elevation()
            elif layer == "solar":
                self._render_solar()
            elif layer == "hazard":
                self._render_hazard()
            elif layer == "path":
                self._render_path()
            elif layer == "comms":
                self._render_comms()
            elif layer == "resources":
                self._render_resources()
            elif layer == "rover":
                self._render_rover()
            elif layer == "psr":
                self._render_psr()
            elif layer == "score":
                self._render_score()
            elif layer == "range":
                self._render_range()

    # -----------------------------------------------------------------------
    # Overlay management
    # -----------------------------------------------------------------------

    def _clear_overlays(self):
        for item in self._overlay_items:
            self._plot_widget.removeItem(item)
        self._overlay_items.clear()

    def _add_overlay(self, item):
        self._plot_widget.addItem(item)
        self._overlay_items.append(item)

    # -----------------------------------------------------------------------
    # Image rendering helper
    # -----------------------------------------------------------------------

    def _show_image(self, data, lut_key, zmin=None, zmax=None, xg=None, yg=None):
        """Display a 2D grid as an ImageItem with the given LUT."""
        if xg is None:
            xg = self._xg
        if yg is None:
            yg = self._yg

        lut = LUTS[lut_key]

        if zmin is None:
            zmin = float(np.nanmin(data))
        if zmax is None:
            zmax = float(np.nanmax(data))

        rng = zmax - zmin
        if rng < 1e-12:
            rng = 1.0

        normed = np.clip((data - zmin) / rng, 0.0, 1.0)
        # Handle NaN as transparent
        nan_mask = np.isnan(data)
        idx = (normed * 255).astype(np.uint8)
        rgba = lut[idx]  # (rows, cols, 4)
        if nan_mask.any():
            rgba[nan_mask, 3] = 0

        # pyqtgraph ImageItem expects (cols, rows) for display with
        # origin at bottom-left, but our data is (rows, cols) with row 0 = y_min.
        # Transpose so axis 0 = x, axis 1 = y.
        self._image_item.setImage(np.transpose(rgba, (1, 0, 2)), levels=None)

        x_min, x_max = float(xg[0, 0]), float(xg[0, -1])
        y_min, y_max = float(yg[0, 0]), float(yg[-1, 0])
        cols, rows = data.shape[1], data.shape[0]
        # setRect(x, y, w, h) in scene/world coordinates
        from PyQt5.QtCore import QRectF

        self._image_item.setRect(QRectF(x_min, y_min, x_max - x_min, y_max - y_min))
        self._plot_widget.setRange(
            xRange=(x_min, x_max), yRange=(y_min, y_max), padding=0.02
        )

    # -----------------------------------------------------------------------
    # Layer renderers
    # -----------------------------------------------------------------------

    def _render_3d(self):
        """3D terrain as a dense scatter plot from the interpolated grid."""
        self._gl_view.clear()
        xg, yg, zg = self._xg, self._yg, self._zg

        # Flatten grid to point array
        pos = np.column_stack([
            xg.ravel(),
            yg.ravel(),
            zg.ravel(),
        ]).astype(np.float32)

        # Colour by elevation using lunar LUT
        z_flat = zg.ravel()
        z_norm = (z_flat - z_flat.min()) / (z_flat.max() - z_flat.min() + 1e-9)
        lut = LUTS["lunar"]
        idx = (z_norm * 255).astype(np.uint8)
        colors = lut[idx].astype(np.float32) / 255.0

        scatter = gl.GLScatterPlotItem(
            pos=pos, color=colors, size=2.0, pxMode=True
        )
        self._gl_view.addItem(scatter)

        # Centre camera on terrain, looking down at an angle
        cx = float(np.mean(xg))
        cy = float(np.mean(yg))
        cz = float(np.mean(zg))
        span = max(float(np.ptp(xg)), float(np.ptp(yg)))
        self._gl_view.opts["center"] = pg.Vector(cx, cy, cz)
        self._gl_view.opts["distance"] = span * 1.5
        self._gl_view.opts["elevation"] = -30
        self._gl_view.opts["azimuth"] = 45
        self._gl_view.update()

    def _render_elevation(self):
        self._show_image(self._zg, "topo")
        if self._chk_contours.isChecked():
            self._draw_contour_lines()

    def _on_contour_toggled(self, checked):
        if self._current_layer == "elevation":
            self._show_layer("elevation")

    def _draw_contour_lines(self):
        """Overlay contour isolines with height labels on the current view."""
        from PyQt5.QtGui import QTransform

        zg = self._zg
        z_min, z_max = float(zg.min()), float(zg.max())
        n_levels = 12
        levels = np.linspace(z_min, z_max, n_levels + 2)[1:-1]
        contour_color = QColor("#000000")
        contour_color.setAlpha(200)

        xg, yg = self._xg, self._yg
        x_min, x_max = float(xg[0, 0]), float(xg[0, -1])
        y_min, y_max = float(yg[0, 0]), float(yg[-1, 0])
        rows, cols = zg.shape
        sx = (x_max - x_min) / (cols - 1) if cols > 1 else 1.0
        sy = (y_max - y_min) / (rows - 1) if rows > 1 else 1.0

        for level in levels:
            iso = pg.IsocurveItem(
                data=zg, level=float(level),
                pen=pg.mkPen(contour_color, width=1.5),
            )
            transform = QTransform()
            transform.translate(x_min, y_min)
            transform.scale(sx, sy)
            iso.setTransform(transform)
            self._add_overlay(iso)

            # Place a height label near the middle of the contour
            # Find a grid cell close to this level near the centre
            diff = np.abs(zg - level)
            # Search in the middle third of the grid
            r3, c3 = rows // 3, cols // 3
            sub = diff[r3:2*r3, c3:2*c3]
            if sub.size > 0:
                idx = np.unravel_index(np.argmin(sub), sub.shape)
                lr, lc = idx[0] + r3, idx[1] + c3
                lx = x_min + lc * sx
                ly = y_min + lr * sy
                txt = pg.TextItem(
                    f"{level:.2f}m",
                    color="#000000",
                    anchor=(0.5, 0.5),
                )
                txt.setFont(QFont("Courier New", 8))
                txt.setPos(lx, ly)
                self._add_overlay(txt)

    def _render_solar(self):
        from scipy.ndimage import gaussian_filter as _gf

        if self._cycle_playing and self._shaded_stack:
            frame = self._cycle_frame % len(self._shaded_stack)
            self._show_image(self._shaded_stack[frame], "shadow", zmin=0.0, zmax=1.0)
        else:
            z_norm = self._z_norm
            illum_smooth = _gf(self._illum0.astype(np.float64), sigma=1.2)
            shaded = z_norm * (0.3 + 0.7 * illum_smooth)
            self._show_image(shaded, "shadow", zmin=0.0, zmax=1.0)

    def _render_hazard(self):
        self._show_image(self._hazard, "hazard", zmin=0.0, zmax=1.0)

    def _render_path(self):
        self._show_image(self._hazard, "hazard", zmin=0.0, zmax=1.0)
        t = _theme(self._theme_name)
        # Draw path
        if self._path_coords and len(self._path_coords) > 0:
            pc = np.asarray(self._path_coords)
            line = pg.PlotDataItem(
                pc[:, 0], pc[:, 1], pen=pg.mkPen(t["accent"], width=3)
            )
            self._add_overlay(line)
        # Start marker
        if self._path_start:
            s = pg.ScatterPlotItem(
                [self._path_start[0]],
                [self._path_start[1]],
                symbol="o",
                size=14,
                brush=pg.mkBrush("#00ff88"),
                pen=pg.mkPen("w", width=2),
            )
            self._add_overlay(s)
        # End marker
        if self._path_end:
            s = pg.ScatterPlotItem(
                [self._path_end[0]],
                [self._path_end[1]],
                symbol="o",
                size=14,
                brush=pg.mkBrush("#cc0000"),
                pen=pg.mkPen("w", width=2),
            )
            self._add_overlay(s)
        # Cost annotation
        if self._path_cost is not None:
            self._info_detail.setText(f"Path cost: {self._path_cost:.1f}")
        elif self._path_start is None:
            self._info_detail.setText("Click to set START point")
        elif self._path_end is None:
            self._info_detail.setText("Click to set END point")

    def _render_comms(self):
        if self._comms_coverage is None:
            return
        self._show_image(self._comms_coverage, "comms", zmin=0.0, zmax=1.0)
        t = _theme(self._theme_name)

        # Shadow boundary contour at 0.15
        xg, yg = self._xg, self._yg
        x_min, x_max = float(xg[0, 0]), float(xg[0, -1])
        y_min, y_max = float(yg[0, 0]), float(yg[-1, 0])
        rows, cols = self._comms_coverage.shape
        iso = pg.IsocurveItem(
            data=self._comms_coverage,
            level=0.15,
            pen=pg.mkPen("#ff4400", width=2, style=Qt.DotLine),
        )
        from PyQt5.QtGui import QTransform

        sx = (x_max - x_min) / (cols - 1) if cols > 1 else 1.0
        sy = (y_max - y_min) / (rows - 1) if rows > 1 else 1.0
        transform = QTransform()
        transform.translate(x_min, y_min)
        transform.scale(sx, sy)
        iso.setTransform(transform)
        self._add_overlay(iso)

        # Base marker
        if self._base_pos:
            s = pg.ScatterPlotItem(
                [self._base_pos[0]],
                [self._base_pos[1]],
                symbol="d",
                size=16,
                brush=pg.mkBrush(t["accent"]),
                pen=pg.mkPen("w", width=2),
            )
            self._add_overlay(s)
            # Label
            txt = pg.TextItem("BASE", color=t["accent"], anchor=(0.5, 1.2))
            txt.setPos(self._base_pos[0], self._base_pos[1])
            self._add_overlay(txt)

        # Rover marker and LOS lines
        if self._rover_pos_comms:
            s = pg.ScatterPlotItem(
                [self._rover_pos_comms[0]],
                [self._rover_pos_comms[1]],
                symbol="o",
                size=12,
                brush=pg.mkBrush(t["font_color"]),
                pen=pg.mkPen("w", width=1),
            )
            self._add_overlay(s)
            txt = pg.TextItem("ROVER", color=t["font_color"], anchor=(0.5, 1.2))
            txt.setPos(self._rover_pos_comms[0], self._rover_pos_comms[1])
            self._add_overlay(txt)

            if self._los_data and self._base_pos:
                bm = self._los_data["blocked_mask"]
                n_seg = len(bm)
                lx = np.linspace(
                    self._base_pos[0], self._rover_pos_comms[0], n_seg + 1
                )
                ly = np.linspace(
                    self._base_pos[1], self._rover_pos_comms[1], n_seg + 1
                )
                for i in range(0, n_seg, 3):
                    color = "#cc0000" if bm[i] else "#00ff88"
                    end = min(i + 4, n_seg)
                    seg = pg.PlotDataItem(
                        [lx[i], lx[end]],
                        [ly[i], ly[end]],
                        pen=pg.mkPen(color, width=3),
                    )
                    self._add_overlay(seg)

                vis = "CLEAR" if self._los_data["visible"] else "BLOCKED"
                self._info_detail.setText(f"LOS: {vis}")

    def _render_resources(self):
        self._show_image(self._ice_prob * 100, "ice", zmin=0.0, zmax=100.0)
        # Drill site markers
        if self._drill_sites:
            ds = np.asarray(self._drill_sites)
            s = pg.ScatterPlotItem(
                ds[:, 0],
                ds[:, 1],
                symbol="star",
                size=14,
                brush=pg.mkBrush("#ff00ff"),
                pen=pg.mkPen("w", width=1),
            )
            self._add_overlay(s)

    def _render_rover(self):
        self._show_image(self._slope_deg, "hazard")
        t = _theme(self._theme_name)

        # Contour lines on slope
        xg, yg = self._xg, self._yg
        x_min, x_max = float(xg[0, 0]), float(xg[0, -1])
        y_min, y_max = float(yg[0, 0]), float(yg[-1, 0])
        rows, cols = self._slope_deg.shape
        from PyQt5.QtGui import QTransform

        sx = (x_max - x_min) / (cols - 1) if cols > 1 else 1.0
        sy = (y_max - y_min) / (rows - 1) if rows > 1 else 1.0

        for level in [5, 10, 15]:
            iso = pg.IsocurveItem(
                data=self._slope_deg,
                level=float(level),
                pen=pg.mkPen("rgba(0,255,170,80)", width=1),
            )
            transform = QTransform()
            transform.translate(x_min, y_min)
            transform.scale(sx, sy)
            iso.setTransform(transform)
            self._add_overlay(iso)

        if self._rover_data:
            # Body outline
            bc = np.asarray(self._rover_data["body_corners"])
            bx = list(bc[:, 0]) + [bc[0, 0]]
            by = list(bc[:, 1]) + [bc[0, 1]]
            body = pg.PlotDataItem(
                bx, by, pen=pg.mkPen(t["accent"], width=2),
                fillLevel=0, fillBrush=pg.mkBrush(0, 204, 255, 60),
            )
            self._add_overlay(body)

            # Wheel markers
            wp = np.asarray(self._rover_data["wheel_positions"])
            contact = self._rover_data["contact_ok"]
            colors = [
                pg.mkBrush("#00ff88") if c else pg.mkBrush("#cc0000")
                for c in contact
            ]
            for i in range(4):
                s = pg.ScatterPlotItem(
                    [wp[i, 0]],
                    [wp[i, 1]],
                    symbol="o",
                    size=10,
                    brush=colors[i],
                    pen=pg.mkPen("w", width=1),
                )
                self._add_overlay(s)

            # Info text
            clr = self._rover_data["min_clearance"]
            self._info_detail.setText(
                f"Clearance: {clr:.2f}m | "
                f"Pitch: {self._rover_data['tilt_pitch']:.1f} deg | "
                f"Roll: {self._rover_data['tilt_roll']:.1f} deg"
            )
        else:
            self._info_detail.setText("Click to place rover")

    def _render_psr(self):
        rad_kwh = self._solar_radiation / 1000.0
        self._show_image(rad_kwh, "solar_rad")

        # PSR boundary contour
        xg, yg = self._xg, self._yg
        x_min, x_max = float(xg[0, 0]), float(xg[0, -1])
        y_min, y_max = float(yg[0, 0]), float(yg[-1, 0])
        rows, cols = self._psr_mask.shape
        psr_float = self._psr_mask.astype(np.float32)
        iso = pg.IsocurveItem(
            data=psr_float, level=0.5, pen=pg.mkPen("#ff00ff", width=2)
        )
        from PyQt5.QtGui import QTransform

        sx = (x_max - x_min) / (cols - 1) if cols > 1 else 1.0
        sy = (y_max - y_min) / (rows - 1) if rows > 1 else 1.0
        transform = QTransform()
        transform.translate(x_min, y_min)
        transform.scale(sx, sy)
        iso.setTransform(transform)
        self._add_overlay(iso)

        # Stats
        psr_count = int(np.sum(self._psr_mask))
        total = self._psr_mask.size
        psr_pct = psr_count / total * 100
        rad_max = float(np.max(rad_kwh)) if np.any(rad_kwh > 0) else 0
        self._info_detail.setText(
            f"PSR: {psr_count:,} cells ({psr_pct:.1f}%)\n"
            f"Peak: {rad_max:.0f} kWh/m^2 | Mean: {float(np.mean(rad_kwh)):.0f} kWh/m^2"
        )

    def _render_score(self):
        self._show_image(
            self._score_grid,
            "score",
            zmin=0.0,
            zmax=100.0,
            xg=self._score_xg,
            yg=self._score_yg,
        )
        s = self._score_summary
        self._info_detail.setText(
            f"Mean: {s['mean_score']:.0f}/100 | Max: {s['max_score']:.0f}/100\n"
            f"Best: ({s['best_location'][0]:.1f}, {s['best_location'][1]:.1f})\n"
            f"Cells >70: {s['coverage_above_70']:.1f}%"
        )

    def _render_range(self):
        t = _theme(self._theme_name)

        if self._lander_pos is None:
            # Show hazard background as placeholder
            self._show_image(self._hazard, "hazard", zmin=0.0, zmax=1.0)
            self._info_detail.setText("Click to place lander")
            return

        # Compute battery range
        lander_key = (
            round(self._lander_pos[0], 2),
            round(self._lander_pos[1], 2),
            round(self._charge_pct, 0),
        )
        if lander_key in self._range_cache:
            cost_to_reach, reachable, range_pct = self._range_cache[lander_key]
        else:
            rows, cols = self._energy_cost_grid.shape
            cost_to_reach, reachable, range_pct = compute_battery_range(
                self._energy_graph,
                rows,
                cols,
                self._xg,
                self._yg,
                self._lander_pos,
                self._charge_pct,
            )
            self._range_cache[lander_key] = (cost_to_reach, reachable, range_pct)

        display = np.clip(range_pct, 0, 100).copy()
        display[~reachable] = np.nan
        self._show_image(display, "range", zmin=0.0, zmax=100.0)

        # Lander marker
        s = pg.ScatterPlotItem(
            [self._lander_pos[0]],
            [self._lander_pos[1]],
            symbol="d",
            size=18,
            brush=pg.mkBrush("#ffaa00"),
            pen=pg.mkPen("w", width=2),
        )
        self._add_overlay(s)
        txt = pg.TextItem("LANDER", color="#ffaa00", anchor=(0.5, 1.2))
        txt.setPos(self._lander_pos[0], self._lander_pos[1])
        self._add_overlay(txt)

        # Waypoints
        if self._waypoints:
            wp = np.asarray(self._waypoints)
            for i in range(len(wp)):
                ws = pg.ScatterPlotItem(
                    [wp[i, 0]],
                    [wp[i, 1]],
                    symbol="o",
                    size=12,
                    brush=pg.mkBrush(t["accent"]),
                    pen=pg.mkPen("w", width=2),
                )
                self._add_overlay(ws)
                wt = pg.TextItem(
                    f"WP{i + 1}", color=t["accent"], anchor=(0.5, 1.2)
                )
                wt.setPos(wp[i, 0], wp[i, 1])
                self._add_overlay(wt)

        # Waypoint route
        if self._wp_path and len(self._wp_path) > 0:
            pp = np.asarray(self._wp_path)
            line = pg.PlotDataItem(
                pp[:, 0], pp[:, 1], pen=pg.mkPen(t["accent"], width=3)
            )
            self._add_overlay(line)

        # Info
        info = (
            f"Charge: {self._charge_pct:.0f}% "
            f"({BATTERY_ENERGY_WH * self._charge_pct / 100:.0f} Wh) | "
            f"Reserve: 25% ({BATTERY_ENERGY_WH * 0.25:.0f} Wh)"
        )
        if self._wp_cost is not None:
            info += f"\nRoute: {self._wp_cost:.1f} Wh"
        self._info_detail.setText(info)

    # -----------------------------------------------------------------------
    # Click handling
    # -----------------------------------------------------------------------

    def _on_plot_hover(self, args):
        """Update the cursor readout panel with all data at the hover position."""
        if self._xg is None:
            return

        pos = args[0]
        vb = self._plot_widget.getPlotItem().getViewBox()
        mouse_point = vb.mapSceneToView(pos)
        wx, wy = float(mouse_point.x()), float(mouse_point.y())

        xg, yg, zg = self._xg, self._yg, self._zg
        x_min, x_max = float(xg[0, 0]), float(xg[0, -1])
        y_min, y_max = float(yg[0, 0]), float(yg[-1, 0])

        if not (x_min <= wx <= x_max and y_min <= wy <= y_max):
            self._hover_label.setText("Cursor outside terrain bounds")
            return

        t = _theme(self._theme_name)
        ac = t["accent"]

        # Sample all grids at this position
        elev = float(_bilinear_interp(zg, xg, yg, wx, wy))

        rows, cols = zg.shape
        col_f = (wx - x_min) / (x_max - x_min + 1e-12) * (cols - 1)
        row_f = (wy - y_min) / (y_max - y_min + 1e-12) * (rows - 1)
        ci = int(np.clip(round(col_f), 0, cols - 1))
        ri = int(np.clip(round(row_f), 0, rows - 1))

        slope = float(self._slope_deg[ri, ci]) if self._slope_deg is not None else 0
        hazard_v = float(self._hazard[ri, ci]) if self._hazard is not None else 0
        solar_up = (
            float(self._solar_uptime[ri, ci]) * 100
            if self._solar_uptime is not None
            else 0
        )
        comms_v = (
            float(self._comms_coverage[ri, ci]) * 100
            if self._comms_coverage is not None
            else 0
        )
        ice_v = (
            float(self._ice_prob[ri, ci]) * 100
            if self._ice_prob is not None
            else 0
        )
        rad_v = (
            float(self._solar_radiation[ri, ci]) / 1000.0
            if self._solar_radiation is not None
            else 0
        )
        psr_v = (
            bool(self._psr_mask[ri, ci])
            if self._psr_mask is not None
            else False
        )

        # Mission score — different grid resolution, needs separate lookup
        score_str = "N/A"
        if self._score_grid is not None:
            sxg, syg = self._score_xg, self._score_yg
            sx_min, sx_max = float(sxg[0, 0]), float(sxg[0, -1])
            sy_min, sy_max = float(syg[0, 0]), float(syg[-1, 0])
            if sx_min <= wx <= sx_max and sy_min <= wy <= sy_max:
                sr, sc = self._score_grid.shape
                sci = int(np.clip(
                    round((wx - sx_min) / (sx_max - sx_min + 1e-12) * (sc - 1)),
                    0, sc - 1,
                ))
                sri = int(np.clip(
                    round((wy - sy_min) / (sy_max - sy_min + 1e-12) * (sr - 1)),
                    0, sr - 1,
                ))
                sv = float(self._score_grid[sri, sci])
                score_str = f"{sv:.0f}/100"
                # Component breakdown
                comp = self._score_comp
                score_str += (
                    f" (Sol:{comp['solar'][sri, sci]:.0f} "
                    f"Slp:{comp['slope'][sri, sci]:.0f} "
                    f"Clr:{comp['clearance'][sri, sci]:.0f} "
                    f"Com:{comp['comms'][sri, sci]:.0f})"
                )

        # Battery range at this point (if lander is placed)
        range_str = "No lander"
        if self._lander_pos is not None:
            lander_key = (
                round(self._lander_pos[0], 2),
                round(self._lander_pos[1], 2),
                round(self._charge_pct, 0),
            )
            if lander_key in self._range_cache:
                _, reachable, range_pct = self._range_cache[lander_key]
                rv = float(range_pct[ri, ci])
                if reachable[ri, ci]:
                    range_str = f"{rv:.0f}% remaining"
                else:
                    range_str = "OUT OF RANGE"

        # Illumination at current frame
        illum_str = ""
        if self._illum0 is not None:
            illum_now = float(self._illum0[ri, ci])
            illum_str = "SUNLIT" if illum_now > 0.5 else "SHADOW"

        # Traversal cost
        cost_str = ""
        if self._cost_grid is not None:
            cv = float(self._cost_grid[ri, ci])
            cost_str = "IMPASSABLE" if np.isinf(cv) else f"{cv:.1f}"

        # Energy cost per metre
        energy_str = ""
        if self._energy_cost_grid is not None:
            ev = float(self._energy_cost_grid[ri, ci])
            energy_str = "IMPASSABLE" if np.isinf(ev) else f"{ev:.4f} Wh/m"

        # Build HTML readout
        lines = [
            f"<b style='color:{ac}'>POSITION</b>",
            f"  X: {wx:.2f} m  Y: {wy:.2f} m",
            f"  Grid: [{ri}, {ci}]",
            "",
            f"<b style='color:{ac}'>TERRAIN</b>",
            f"  Elevation: {elev:.2f} m",
            f"  Slope:     {slope:.1f} deg",
            f"  Hazard:    {hazard_v:.2f}",
            f"  Traversal: {cost_str}",
            "",
            f"<b style='color:{ac}'>SOLAR</b>",
            f"  Uptime:    {solar_up:.0f}%",
            f"  Radiation: {rad_v:.1f} kWh/m^2",
            f"  Current:   {illum_str}",
            f"  PSR:       {'YES' if psr_v else 'No'}",
            "",
            f"<b style='color:{ac}'>COMMS</b>",
            f"  Signal:    {comms_v:.0f}%",
            "",
            f"<b style='color:{ac}'>RESOURCES</b>",
            f"  Ice prob:  {ice_v:.1f}%",
            "",
            f"<b style='color:{ac}'>MISSION</b>",
            f"  Score:     {score_str}",
            "",
            f"<b style='color:{ac}'>BATTERY</b>",
            f"  Energy:    {energy_str}",
            f"  Range:     {range_str}",
        ]
        self._hover_label.setText("<pre>" + "\n".join(lines) + "</pre>")

    def _on_plot_clicked(self, event):
        """Handle mouse clicks on the 2D plot view."""
        if self._xg is None:
            return

        # Map scene coordinates to data coordinates
        pos = event.scenePos()
        vb = self._plot_widget.getPlotItem().getViewBox()
        mouse_point = vb.mapSceneToView(pos)
        wx, wy = float(mouse_point.x()), float(mouse_point.y())

        # Check bounds
        x_min = float(self._xg[0, 0])
        x_max = float(self._xg[0, -1])
        y_min = float(self._yg[0, 0])
        y_max = float(self._yg[-1, 0])
        if not (x_min <= wx <= x_max and y_min <= wy <= y_max):
            return

        # Sidebar click mode takes priority over layer-specific clicks
        if self._click_mode == "lander":
            self._place_lander(wx, wy)
            return
        elif self._click_mode == "rover":
            self._place_rover(wx, wy)
            return
        elif self._click_mode == "wp":
            self._place_waypoint(wx, wy)
            return

        # Layer-specific click behaviour
        layer = self._current_layer
        if layer == "path":
            self._handle_path_click(wx, wy)
        elif layer == "comms":
            self._handle_comms_click(wx, wy)
        elif layer == "rover":
            self._handle_rover_click(wx, wy)
        elif layer == "range":
            self._handle_range_click(wx, wy)

    def _handle_path_click(self, wx, wy):
        if self._path_start is None:
            self._path_start = (wx, wy)
            self._path_end = None
            self._path_coords = None
            self._path_cost = None
            self._info_detail.setText(
                f"Start: ({wx:.2f}, {wy:.2f}) — click for END"
            )
        elif self._path_end is None:
            self._path_end = (wx, wy)
            self._info_detail.setText("Computing path...")
            QApplication.processEvents()

            path_coords, cost = find_path(
                self._traversal_graph,
                self._cost_grid,
                self._xg,
                self._yg,
                self._path_start,
                self._path_end,
            )
            if path_coords:
                self._path_coords = path_coords
                self._path_cost = cost
            else:
                self._info_detail.setText("No path found!")
            # Reset for next click pair
            self._path_start_next = True
        else:
            # Reset
            self._path_start = (wx, wy)
            self._path_end = None
            self._path_coords = None
            self._path_cost = None
        self._show_layer("path")

    def _handle_comms_click(self, wx, wy):
        # On comms layer, direct clicks check LOS from base to clicked point
        self._rover_pos_comms = (wx, wy)
        self._rover_pos = (wx, wy)
        if self._base_pos:
            self._los_data = compute_line_of_sight(
                self._xg, self._yg, self._zg,
                self._base_pos, self._rover_pos_comms,
            )
        self._update_position_labels()
        self._show_layer("comms")

    def _handle_rover_click(self, wx, wy):
        self._place_rover(wx, wy)

    def _handle_range_click(self, wx, wy):
        self._place_waypoint(wx, wy)

    # -----------------------------------------------------------------------
    # Unified placement handlers (sidebar buttons, work on any layer)
    # -----------------------------------------------------------------------

    def _place_lander(self, wx, wy):
        """Place lander/base station — shared by comms and battery layers."""
        self._base_pos = (wx, wy)
        self._lander_pos = (wx, wy)
        self._range_cache.clear()
        self._waypoints.clear()
        self._wp_path = None
        self._wp_cost = None

        # Recompute comms coverage from new base position
        self._info_detail.setText("Recomputing coverage...")
        QApplication.processEvents()
        self._comms_coverage = compute_comms_coverage(
            self._xg, self._yg, self._zg, self._base_pos
        )
        self._cost_grid = compute_traversal_cost(
            self._slope_deg, self._hazard, self._solar_uptime, self._comms_coverage
        )
        self._traversal_graph = build_sparse_graph(
            self._cost_grid, self._xg, self._yg
        )
        # Recompute scores with new comms
        (
            self._score_xg,
            self._score_yg,
            self._score_grid,
            self._score_comp,
            self._score_summary,
        ) = compute_mission_score(
            self._xg, self._yg, self._zg,
            self._slope_deg, self._solar_uptime, self._comms_coverage,
            self._hazard, self._ice_prob, cell_size=0.25,
        )

        self._update_position_labels()
        self._info_detail.setText(
            f"Lander placed at ({wx:.2f}, {wy:.2f})"
        )
        self._show_layer(self._current_layer)

    def _place_rover(self, wx, wy):
        """Place rover — updates footprint on any layer."""
        self._rover_pos = (wx, wy)
        self._rover_pos_comms = (wx, wy)
        self._rover_data = compute_rover_footprint(
            self._xg, self._yg, self._zg,
            self._rover_pos, rover_heading=self._rover_heading,
        )
        # Compute LOS from base to rover
        if self._base_pos:
            self._los_data = compute_line_of_sight(
                self._xg, self._yg, self._zg,
                self._base_pos, self._rover_pos,
            )
        self._update_position_labels()
        self._info_detail.setText(
            f"Rover placed at ({wx:.2f}, {wy:.2f})"
        )
        self._show_layer(self._current_layer)

    def _place_waypoint(self, wx, wy):
        """Add a battery range waypoint."""
        if self._lander_pos is None:
            self._info_detail.setText("Place lander first!")
            return
        self._waypoints.append((wx, wy))
        # Recompute route
        wp_path, wp_cost = compute_wp_route(
            self._lander_pos, self._waypoints,
            self._traversal_graph, self._cost_grid,
            self._xg, self._yg,
        )
        self._wp_path = wp_path
        self._wp_cost = wp_cost
        self._update_position_labels()
        self._info_detail.setText(
            f"WP{len(self._waypoints)} at ({wx:.2f}, {wy:.2f})"
        )
        self._show_layer(self._current_layer)

    # -----------------------------------------------------------------------
    # Sidebar control handlers
    # -----------------------------------------------------------------------

    def _on_apply_location(self):
        """Recompute all solar-dependent layers with the new lat/lon."""
        lat = self._global_lat.value()
        lon = self._global_lon.value()
        self._sun_lat = lat
        self._sun_lon = lon
        # Sync the sun controls spinboxes too
        self._spin_lat.setValue(lat)
        self._spin_lon.setValue(lon)

        self._location_info.setText(f"Recomputing for ({lat:.1f}, {lon:.1f})...")
        QApplication.processEvents()

        # Recompute current shadow
        sun_dir = compute_sun_direction(self._sun_date, lat, lon)
        self._illum0 = compute_shadow_map(self._xg, self._yg, self._zg, sun_dir)

        # Recompute full lunar cycle
        (
            self._illum_stack,
            self._cycle_ts,
            self._solar_uptime,
            self._solar_radiation,
            self._psr_mask,
        ) = compute_lunar_cycle_illumination(
            self._xg, self._yg, self._zg, self._sun_date,
            lat, lon, n_steps=112,
        )
        from scipy.ndimage import gaussian_filter as _gf

        self._z_norm = (self._zg - self._zg.min()) / (
            self._zg.max() - self._zg.min() + 1e-9
        )
        self._shaded_stack = []
        self._shaded_ts = []
        for i, il in enumerate(self._illum_stack):
            if float(np.mean(il)) > 0.01:
                smooth = _gf(il.astype(np.float64), sigma=1.2)
                self._shaded_stack.append(self._z_norm * (0.3 + 0.7 * smooth))
                if self._cycle_ts:
                    self._shaded_ts.append(self._cycle_ts[i])
        if not self._shaded_stack:
            self._shaded_stack = [
                self._z_norm * (0.3 + 0.7 * _gf(il.astype(np.float64), sigma=1.2))
                for il in self._illum_stack
            ]
            self._shaded_ts = list(self._cycle_ts) if self._cycle_ts else []

        # Recompute ice deposits (depends on illumination)
        self._ice_prob, self._drill_sites = generate_ice_deposits(
            self._xg, self._yg, self._zg, self._illum0,
        )

        # Recompute mission scores
        (
            self._score_xg, self._score_yg, self._score_grid,
            self._score_comp, self._score_summary,
        ) = compute_mission_score(
            self._xg, self._yg, self._zg, self._slope_deg,
            self._solar_uptime, self._comms_coverage,
            self._hazard, self._ice_prob, cell_size=0.25,
        )

        self._location_info.setText(f"({lat:.1f}, {lon:.1f})")
        self._show_layer(self._current_layer)

    def _on_update_sun(self):
        self._sun_lat = self._spin_lat.value()
        self._sun_lon = self._spin_lon.value()
        hour = self._slider_hour.value()
        qd = self._date_edit.date()
        self._sun_date = datetime(
            qd.year(), qd.month(), qd.day(), hour, tzinfo=timezone.utc
        )
        sun_dir = compute_sun_direction(self._sun_date, self._sun_lat, self._sun_lon)
        self._illum0 = compute_shadow_map(self._xg, self._yg, self._zg, sun_dir)

        elev = math.degrees(math.asin(np.clip(sun_dir[2], -1, 1)))
        az = math.degrees(math.atan2(sun_dir[0], sun_dir[1])) % 360
        status = "ABOVE" if sun_dir[2] > 0 else "BELOW"
        pct = float(np.mean(self._illum0) * 100)
        self._info_detail.setText(
            f"Sun Az: {az:.1f} | Elev: {elev:.2f} | {status} | {pct:.0f}% lit"
        )
        self._cycle_label.setText(
            f"{self._sun_date.strftime('%Y-%m-%d %H:%M')} UTC"
        )

        if self._current_layer == "solar":
            self._show_layer("solar")

    def _toggle_cycle(self):
        if self._cycle_playing:
            self._cycle_playing = False
            self._cycle_timer.stop()
            self._btn_cycle.setText("Play Lunar Day (20s)")
            self._cycle_label.setText("")
        else:
            self._cycle_playing = True
            self._cycle_frame = 0
            self._btn_cycle.setText("Stop")
            # 112 frames over 20 seconds = ~178ms per frame
            n = len(self._shaded_stack) if self._shaded_stack else 112
            interval = max(50, int(20000 / n))
            self._cycle_timer.setInterval(interval)
            self._cycle_timer.start()

    def _cycle_step(self):
        if not self._shaded_stack:
            return
        n = len(self._shaded_stack)
        self._cycle_frame = self._cycle_frame + 1

        # Loop back to start
        if self._cycle_frame >= n:
            self._cycle_frame = 0

        # Update display
        pct = float(np.mean(self._shaded_stack[self._cycle_frame]) * 100)
        frame_label = f"Frame {self._cycle_frame + 1}/{n} | {pct:.0f}% sunlit"
        if self._shaded_ts and self._cycle_frame < len(self._shaded_ts):
            ts = self._shaded_ts[self._cycle_frame]
            frame_label = ts.strftime("%b %d %H:%M") + f" | {pct:.0f}% sunlit"
        self._cycle_label.setText(frame_label)

        if self._current_layer == "solar":
            self._show_image(
                self._shaded_stack[self._cycle_frame], "shadow", zmin=0.0, zmax=1.0
            )
            if self._shaded_ts and self._cycle_frame < len(self._shaded_ts):
                ts = self._shaded_ts[self._cycle_frame]
                self._info_detail.setText(
                    ts.strftime("%Y-%m-%d %H:%M UTC") + f" | {pct:.0f}% sunlit"
                )

    def _on_heading_changed(self, value):
        self._rover_heading = value
        self._heading_label.setText(f"{value} deg")
        if self._rover_pos:
            self._rover_data = compute_rover_footprint(
                self._xg,
                self._yg,
                self._zg,
                self._rover_pos,
                rover_heading=value,
            )
            self._show_layer(self._current_layer)

    def _on_charge_changed(self, value):
        self._charge_pct = float(value)
        self._charge_label.setText(f"{value}%")
        self._range_cache.clear()
        if self._lander_pos:
            self._show_layer(self._current_layer)

    def _on_power_model_changed(self):
        """Recompute energy cost grid when power consumption sliders change."""
        current_flat = self._slider_current_flat.value() / 10.0
        current_max = self._slider_current_max.value() / 10.0
        speed = self._slider_speed.value() / 100.0

        # Update labels
        self._current_flat_label.setText(f"{current_flat:.1f} A")
        self._current_max_label.setText(f"{current_max:.1f} A")
        self._speed_label.setText(f"{speed:.2f} m/s")

        # Ensure flat <= max
        if current_flat > current_max:
            current_flat = current_max

        # Recompute energy cost grid with custom parameters
        from lunar_pcd_compute import BATTERY_VOLTAGE

        difficulty = np.clip(
            np.maximum(self._slope_deg / 15.0, self._hazard), 0.0, 1.0
        )
        current = current_flat + difficulty * (current_max - current_flat)
        power_w = BATTERY_VOLTAGE * current
        wh_per_metre = power_w / speed / 3600.0
        wh_per_metre[self._slope_deg > 20.0] = np.inf
        self._energy_cost_grid = wh_per_metre

        # Rebuild energy graph
        self._energy_graph = build_sparse_graph(
            self._energy_cost_grid, self._xg, self._yg
        )
        self._range_cache.clear()

        self._info_detail.setText(
            f"Power model: {current_flat:.1f}-{current_max:.1f} A @ {speed:.2f} m/s"
        )
        if self._lander_pos:
            self._show_layer(self._current_layer)

    def _on_clear_waypoints(self):
        self._waypoints.clear()
        self._wp_path = None
        self._wp_cost = None
        self._update_position_labels()
        self._show_layer(self._current_layer)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def main():
    parser = argparse.ArgumentParser(
        description="Perseus Lunar PCD Viewer (Qt Desktop)"
    )
    parser.add_argument("pcd_file", help="Path to .pcd file")
    args = parser.parse_args()

    app = QApplication(sys.argv)
    app.setApplicationName("Perseus Lunar PCD Viewer")

    viewer = LunarPCDViewer(args.pcd_file)
    viewer.show()

    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
