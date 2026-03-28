# Perseus Lunar PCD Utilities

Tools for visualising and analysing lunar LiDAR point cloud data (.pcd files).

All dependencies (`PyOpenGL`, `pyqtgraph`, `open3d`, `plotly`, `dash`, etc.) are
provided by the Nix dev shell — no pip installs required.

## Quick start

Enter the Nix dev shell, then run from `software/utilities/`:

```bash
nix develop   # or direnv if configured

# Desktop viewer (recommended)
nixgl python3 lunar_pcd_viewer_qt.py scan_1.pcd

# Browser viewer
python3 lunar_pcd_viewer.py scan_1.pcd --port 8060
```

## Viewers

### Desktop viewer (PyQt5/pyqtgraph)

```bash
nixgl python3 lunar_pcd_viewer_qt.py <path_to.pcd>
```

Native Qt window with 11 analysis layers including 3D point cloud rendering.
Requires `nixgl` for OpenGL context (see below).

### Browser viewer (Dash/Plotly)

```bash
python3 lunar_pcd_viewer.py <path_to.pcd> --port 8060
```

Opens a Dash web app at `http://localhost:8060`. Does not require `nixgl`.

## Why `nixgl`?

On non-NixOS systems, OpenGL drivers from the host aren't visible inside the
Nix shell. The `nixgl` wrapper bridges the host GPU drivers so PyQt5/OpenGL can
create a rendering context.

- **With `nixgl`** — all 11 layers work, including the 3D point cloud scatter view.
- **Without `nixgl`** — the 10 2D layers work normally. The 3D layer shows a
  fallback message instead of crashing.

## Shared compute module

Both viewers import `lunar_pcd_compute.py` for all terrain analysis:

- Terrain grid interpolation and slope/hazard maps
- 28-day solar illumination cycle and shadow casting
- Comms line-of-sight and RF coverage
- Dijkstra path planning with energy-aware traversal costs
- Rover footprint simulation
- Battery range planning
- Mission suitability scoring

## Available PCD files

- `scan_1.pcd`, `scan_2.pcd` — rover LiDAR scans
- `lunar_south_pole.pcd` — Shackleton Crater region
- `apollo15_landing_site.pcd`, `apollo15_landing_site_2km.pcd` — Apollo 15 terrain
