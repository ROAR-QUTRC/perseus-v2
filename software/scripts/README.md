# Perseus Lunar PCD Scripts

Tools for visualising and analysing lunar LiDAR point cloud data (.pcd files).

## Viewers

### Browser viewer (Dash/Plotly)

```bash
python3 lunar_pcd_viewer.py scan_1.pcd --port 8060
```

Opens a Dash web app at `http://localhost:8060`.

### Desktop viewer (PyQt5/pyqtgraph)

```bash
nixgl python3 lunar_pcd_viewer_qt.py scan_1.pcd
```

Native Qt window with the same 12 analysis layers as the browser version.

## Why `nixgl`?

On NixOS (and Nix-managed environments), OpenGL drivers from the host system
aren't visible inside the Nix shell by default. The `nixgl` wrapper sets up the
correct driver paths so that PyQt5/OpenGL can create a rendering context.

- **With `nixgl`** — all 12 layers work, including the 3D point cloud scatter view.
- **Without `nixgl`** — the 11 2D layers work normally. The 3D layer shows a
  fallback message instead of crashing.

If you don't have `nixgl` available, install it or add it to your Nix flake:

```nix
# In your flake inputs or shell packages
nixgl.url = "github:nix-community/nixGL";
```

Then run with:

```bash
nixgl python3 lunar_pcd_viewer_qt.py scan_1.pcd
```

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
